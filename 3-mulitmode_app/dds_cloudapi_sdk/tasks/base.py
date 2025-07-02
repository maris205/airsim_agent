import abc
import enum
import importlib
import importlib.metadata
import logging
import time
import uuid

import requests
import sentry_sdk
from flask import json

from dds_cloudapi_sdk.config import Config

logger = logging.getLogger("dds_cloudapi_sdk")
if sentry_sdk.get_client() is None or not sentry_sdk.get_client().is_active():
    sentry_sdk.init(
        server_name="dds_cloudapi_sdk",
        dsn="https://f05b1518ce3d40c8b41f1483c30c46b6@sentry.cvrgo.com/25",
        send_default_pii=True,
        traces_sample_rate=1.0,
        release=importlib.metadata.version("dds-cloudapi-sdk"),
        auto_enabling_integrations=False,
        in_app_include=["dds_cloudapi_sdk"],
    )
http_session = requests.Session()


class TaskStatus(enum.Enum):
    Triggering = "triggering"  # send request to
    Waiting = "waiting"  # wait for server to run this task
    Running = "running"  # server is running this task
    Success = "success"  # task is completed successfully
    Failed = "failed"  # task is failed


class LabelTypes(enum.Enum):
    BBox = "bbox"
    Mask = "mask"


class ErrCode:
    Retry = 202001


class Retry(Exception):
    pass


class BaseTask(abc.ABC):

    _request_timeout = 5

    def __init__(self):
        super().__init__()

        self.config = None
        self.task_uuid = None
        self.status = None
        self.error = None
        self._result = None
        self.trigger_idempotency_key = uuid.uuid4().hex

    @property
    @abc.abstractmethod
    def api_path(self):
        raise NotImplementedError

    @property
    @abc.abstractmethod
    def api_body(self):
        raise NotImplementedError

    @abc.abstractmethod
    def format_result(self, result: dict) -> dict:
        raise NotImplementedError

    @property
    @abc.abstractmethod
    def result(self):
        raise NotImplementedError

    @property
    def headers(self):
        return {"Token": self.config.token}

    @property
    def trigger_headers(self):
        return {"Token": self.config.token, "Idempotency-Key": self.trigger_idempotency_key, "Content-Type": "application/json"}

    @property
    def api_trigger_url(self):
        return f"https://{self.config.endpoint}/tasks/{self.api_path}"

    @property
    def api_check_url(self):
        return f"https://{self.config.endpoint}/task_statuses/{self.task_uuid}"

    def set_request_timeout(self, timeout):
        self._request_timeout = timeout

    def trigger(self, config: Config):
        if self.no_need_to_trigger():
            return

        self.config = config
        self.status = TaskStatus.Triggering
        payload = json.dumps(self.api_body)

        sentry_sdk.set_extra("request-size", len(payload))
        rsp = http_session.post(
            self.api_trigger_url,
            data=payload,
            headers=self.trigger_headers,
            timeout=self._request_timeout
        )
        rsp_json = rsp.json()
        sentry_sdk.set_extra("response-size", len(rsp.content))
        if rsp_json["code"] == ErrCode.Retry:
            raise Retry(f"Failed to trigger {self}, error: {rsp_json['msg']}")
        if rsp_json["code"] != 0:
            raise RuntimeError(f"Failed to trigger {self}, error: {rsp_json['msg']}")
        self.task_uuid = rsp_json["data"]["task_uuid"]

        logger.info(f"{self} is triggered successfully")

    def no_need_to_trigger(self):
        return self.status in (TaskStatus.Success, TaskStatus.Failed, TaskStatus.Waiting, TaskStatus.Running)

    def check(self):
        if self.status is None:
            raise RuntimeError(f"{self} is not triggered, you can't check it's status")

        api = self.api_check_url
        rsp = http_session.get(api, timeout=self._request_timeout, headers=self.headers)
        rsp_json = rsp.json()
        if rsp_json["code"] != 0:
            raise RuntimeError(f"Failed to check {self}, error: {rsp_json['msg']}")

        task_data = rsp_json["data"]
        self.status = TaskStatus(task_data["status"])
        if self.status == TaskStatus.Success:
            result = task_data["result"]
            self._result = self.format_result(result)
        elif self.status == TaskStatus.Failed:
            self.error = task_data["error"]

    def wait(self):
        if self.status is None:
            raise RuntimeError(f"{self} is not triggered, you can't wait for it's result")

        while True:
            if self.status not in {TaskStatus.Triggering, TaskStatus.Waiting, TaskStatus.Running}:
                return

            self.check()
            if self.status == TaskStatus.Waiting:
                logger.info(f"{self} is waiting")
            elif self.status == TaskStatus.Running:
                logger.info(f"{self}  is running")
            elif self.status == TaskStatus.Success:
                logger.info(f"{self}  is success")
                return
            elif self.status == TaskStatus.Failed:
                logger.info(f"{self}  is failed")
                raise RuntimeError(f"{self}  is failed, error: {self.error}")
            time.sleep(0.5)

    def run(self, config: Config):
        for i in range(3):
            try:
                with sentry_sdk.start_transaction(op="trigger_task", name=f"{self.api_path}"):
                    sentry_sdk.set_tag("model", self.api_body.get("model", ""))
                    sentry_sdk.set_tag("token", config.token)
                    self.trigger(config)
                self.wait()
                return
            except (Retry, requests.exceptions.ReadTimeout) as e:
                logger.warning(f"Failed to trigger {self}, times: {i+1}, e:{e}")
                if i < 2:
                    time.sleep(2)
                    continue
                sentry_sdk.capture_exception(e)
                raise e
            except Exception as e:
                sentry_sdk.capture_exception(e)
                raise

    def __str__(self):
        return f"{self.__class__.__name__}<task_id:{self.task_uuid}, idemp_key:{self.trigger_idempotency_key}>"
