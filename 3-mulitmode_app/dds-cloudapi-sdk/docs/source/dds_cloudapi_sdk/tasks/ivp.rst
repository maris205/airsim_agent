.. currentmodule:: dds_cloudapi_sdk.tasks.ivp

IVP - Interactive Visual Prompt
===============================

.. automodule:: dds_cloudapi_sdk.tasks.ivp
   :no-members:

Usage Pattern
-------------

First of all, make sure you have installed this SDK by `pip`::

    pip install dds-cloudapi-sdk

Then trigger the algorithm through IVPTask::

    from dds_cloudapi_sdk import Config
    from dds_cloudapi_sdk import Client
    from dds_cloudapi_sdk import IVPTask
    from dds_cloudapi_sdk import RectPrompt
    from dds_cloudapi_sdk import LabelTypes

    # Step 1: initialize the config
    token = "Your API token here"
    config = Config(token)

    # Step 2: initialize the client
    client = Client(config)

    # Step 3: run the task by IVPTask class

    prompt_image_url = "https://dds-frontend.oss-cn-shenzhen.aliyuncs.com/static_files/playground/grounded_sam/05.jpg"
    # if you are processing local image file, upload them to DDS server to get the image url
    # prompt_image_url = client.upload_file("/path/to/your/prompt/image.png")

    # use the same image for inferring
    infer_image_url = prompt_image_url

    task = IVPTask(
        prompt_image_url=prompt_image_url,
        prompts=[RectPrompt(rect=[475.18413597733706, 550.1983002832861, 548.1019830028329, 599.915014164306])],
        infer_image_url=infer_image_url,
        infer_label_types=[LabelTypes.BBox, LabelTypes.Mask],  # detect both bbox and mask
    )

    client.run_task(task)
    result = task.result

    print(result.mask_url)

    objects = result.objects  # the list of detected objects
    for idx, obj in enumerate(objects):
        print(obj.score)  # 0.42

        print(obj.bbox)  # [635.0, 458.0, 704.0, 508.0]

        print(obj.mask.counts)  # RLE compressed to string, ]o`f08fa14M3L2O2M2O1O1O1O1N2O1N2O1N2N3M2O3L3M3N2M2N3N1N2O...

        # convert the RLE format to RGBA image
        mask_image = task.rle2rgba(obj.mask)
        print(mask_image.size)  # (1600, 1170)

        # save the image to file
        mask_image.save(f"data/mask_{idx}.png")


API Reference
-------------

.. autoclass:: IVPTask
   :members: result
   :exclude-members: __init__,format_result
   :inherited-members:

.. autoclass:: dds_cloudapi_sdk.tasks.prompt.RectPrompt
   :members:
   :exclude-members: model_computed_fields,model_config,model_fields

.. autoclass:: TaskResult
   :members:
   :exclude-members: model_computed_fields,model_config,model_fields

.. autoclass:: IVPObject
   :members:
   :exclude-members: model_computed_fields,model_config,model_fields

.. autoclass:: IVPObjectMask
   :members:
   :exclude-members: model_computed_fields,model_config,model_fields
