.. currentmodule:: dds_cloudapi_sdk.tasks.dinox

Detection - DINO-X
===============================

.. automodule:: dds_cloudapi_sdk.tasks.dinox
   :no-members:

Usage Pattern
-------------

First of all, make sure you have installed this SDK by `pip`::

    pip install dds-cloudapi-sdk

Then trigger the algorithm through DetectionTask::

    from dds_cloudapi_sdk import Config
    from dds_cloudapi_sdk import Client
    from dds_cloudapi_sdk import TextPrompt
    from dds_cloudapi_sdk.task.dinox import DinoxTask

    # Step 1: initialize the config
    token = "Your API token here"
    config = Config(token)

    # Step 2: initialize the client
    client = Client(config)

    # Step 3: run the task by DetectionTask class
    image_url = "https://algosplt.oss-cn-shenzhen.aliyuncs.com/test_files/tasks/dinox/08.jpg"
    # if you are processing local image file, upload them to DDS server to get the image url
    # image_url = client.upload_file("/path/to/your/prompt/image.png")

    task = DinoxTask(
        image_url=image_url,
        prompts=[TextPrompt(text="<prompt_free>")] # or specific prompts like 'vessel.cutting.person'
    )

    client.run_task(task)
    result = task.result


    objects = result.objects  # the list of detected objects
    for idx, obj in enumerate(objects):

        print(obj.category)  # "person"

        print(obj.bbox)  # [132.2875213623047, 4.497652053833008, 444.68719482421875, 530.9923706054688]

        print(obj.mask.counts)  # RLE compressed to string, ]o`gg0=[95K3M4L4M3L4M2N3L3N2N3M2N3M2N2N2N2N2M3N3M2N3L3N2N3M2N2N2N2O1N2N2N2O0O2N...

        print(obj.pose) # [307.07562255859375, 140.30242919921875, 1, 0, 318.3280029296875, 124.33451080322266, 1,0 ...]

        print(obj.hand) # null

        break


API Reference
-------------
.. autoclass:: dds_cloudapi_sdk.tasks.dinox.DinoxTask
   :members: result
   :exclude-members: __init__,format_result
   :inherited-members:

.. autoclass:: dds_cloudapi_sdk.tasks.prompt.TextPrompt
   :members:
   :exclude-members: model_computed_fields,model_config,model_fields

.. autoclass:: TaskResult
   :members:
   :exclude-members: model_computed_fields,model_config,model_fields

