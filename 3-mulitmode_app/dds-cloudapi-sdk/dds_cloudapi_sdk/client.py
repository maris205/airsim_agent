"""
Client is the main entry point for users to interact with the DDS Cloud API.
After initializing it with a  :class:`Config <dds_cloudapi_sdk.config.Config>` class, users can use it to upload files,
trigger tasks, and check for the results.

A simple example illustrating the major interface::

    from dds_cloudapi_sdk import Config
    from dds_cloudapi_sdk import Client

    token = "Your API Token Here"
    config = Config(token)
    client = Client(config)

    # run a task with client
    client.run(task)
    print(task.result)

"""

import os.path

import requests

from dds_cloudapi_sdk.config import Config
from dds_cloudapi_sdk.tasks.base import BaseTask

__all__ = [
    "Client"
]


class Client:
    """
    | This is the SDK client for dds cloud APIs.
    | It is initialized with the API token, and talks to the server to:

    - 1. upload files to get the visible url
    - 2. run tasks and wait for the results

    :param config: The :class:`Config <dds_cloudapi_sdk.config.Config>` object.

    """

    def __init__(self, config: Config):
        self.config = config

    def trigger_task(self, task: BaseTask):
        """
        Trigger a task and return immediately without waiting for the result.

        :param task: The task to trigger.
        """
        return task.trigger(self.config)

    def check_task(self, task: BaseTask):
        """
        Check the task's :class:`status <dds_cloudapi_sdk.tasks.base.TaskStatus>`.

        :param task: The task to check.
        """
        return task.check()

    def wait_task(self, task: BaseTask):
        """
        | Wait for the task to complete.
        | This blocks the current thread until the task is done.

        :param task: The task to wait.
        """
        return task.wait()

    def run_task(self, task: BaseTask):
        """
        | Trigger a task and wait for it to complete.
        | This blocks the current thread until the task is done.

        :param task: The task to run.
        """
        return task.run(self.config)
