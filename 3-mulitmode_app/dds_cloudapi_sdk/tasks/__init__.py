"""
Each task class in the SDK represents algorithm, offering a unified interface to execute different algorithms.

Here's how to utilize a task class effectively:

- Initialize the task object::

    from dds_cloudapi_sdk.tasks import IVPTask

    task = IVPTask(...)  # take IVP for example, parameters are omitted for brevity

- Run the task with the client object::

    client.run(task)  # steps to initialize a client are omitted for brevity

- Get the result from **task.result** property::

    print(task.result)

"""

from dds_cloudapi_sdk.tasks.base import LabelTypes
from dds_cloudapi_sdk.tasks.base import TaskStatus

__all__ = [
    "TaskStatus",
    "LabelTypes",
]
