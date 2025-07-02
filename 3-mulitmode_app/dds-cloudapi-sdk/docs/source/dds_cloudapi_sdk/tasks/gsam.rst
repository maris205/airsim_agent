.. currentmodule:: dds_cloudapi_sdk.tasks.gsam

GSAM - Grounded SegmentAnyThing
===============================

.. automodule:: dds_cloudapi_sdk.tasks.gsam
   :no-members:

Usage Pattern
-------------
This section demonstrates the usages of both TinyGSAMTask and BaseGSAMTask.

TinyGSAMTask
~~~~~~~~~~~~

First of all, make sure you have installed this SDK by `pip`::

    pip install dds-cloudapi-sdk

Then trigger the Grounded-SAM algorithm with tiny model by TinyGSAMTask::

    from dds_cloudapi_sdk import Client
    from dds_cloudapi_sdk import Config
    from dds_cloudapi_sdk import TextPrompt
    from dds_cloudapi_sdk import TinyGSAMTask

    # Step 1: initialize the config
    token = "Your API token here"
    config = Config(token)

    # Step 2: initialize the client
    client = Client(config)

    # Step 3: run the task by TinyGSAMTask class

    image_url = "https://dds-frontend.oss-cn-shenzhen.aliyuncs.com/static_files/playground/grounded_sam/05.jpg"
    # if you are processing local image file, upload them to DDS server to get the image url
    # image_url = client.upload_file("/path/to/your/image.png")

    task = TinyGSAMTask(
        image_url=image_url,
        prompts=[TextPrompt(text="iron man")]
    )

    client.run_task(task)
    print(task.result.mask_url)  # https://host.com/image.png

    for obj in task.result.objects:
        print(obj.category)  # iron man
        print(obj.score)  # 0.49
        print(obj.bbox)  # [653.08, 329.13, 942.05, 842.50]


BaseGSAMTask
~~~~~~~~~~~~

The usage pattern of BaseGSAMTask is exactly the same like TinyGSAMTask, except that it triggers the algorithm with a different task class::

    # install the SDK by pip
    pip install dds-cloudapi-sdk

Then trigger the task using the BaseGSAMTask class::

    from dds_cloudapi_sdk import Config
    from dds_cloudapi_sdk import Client
    from dds_cloudapi_sdk import TextPrompt
    from dds_cloudapi_sdk import BaseGSAMTask

    # Step 1: initialize the config
    token = "Your API token here"
    config = Config(token)

    # Step 2: initialize the client
    client = Client(config)

    # Step 3: run the task by BaseGSAMTask class

    image_url = "https://dds-frontend.oss-cn-shenzhen.aliyuncs.com/static_files/playground/grounded_sam/05.jpg"
    # if you are processing local image file, upload them to DDS server to get the image url
    # image_url = client.upload_file("/path/to/your/image.png")

    task = BaseGSAMTask(
        image_url=image_url,
        prompts=[TextPrompt(text="iron man")]
    )

    client.run_task(task)
    print(task.result.mask_url)  # https://host.com/image.png

    for obj in task.result.objects:
        print(obj.category)  # iron man
        print(obj.score)  # 0.49
        print(obj.bbox)  # [653.08, 329.13, 942.05, 842.50]


API Reference
-------------

.. autoclass:: TinyGSAMTask
   :members: result
   :exclude-members: __init__,format_result
   :inherited-members:

.. autoclass:: BaseGSAMTask
   :members:
   :exclude-members: __init__,format_result
   :inherited-members:

.. autoclass:: dds_cloudapi_sdk.tasks.prompt.TextPrompt
   :members:
   :exclude-members: model_computed_fields,model_config,model_fields

.. autoclass:: TaskResult
   :members:
   :exclude-members: model_computed_fields,model_config,model_fields

.. autoclass:: GSAMObject
   :members:
   :exclude-members: model_computed_fields,model_config,model_fields
