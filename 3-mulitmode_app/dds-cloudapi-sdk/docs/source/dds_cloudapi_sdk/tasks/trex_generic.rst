.. currentmodule:: dds_cloudapi_sdk.tasks.trex_generic

Trex - Generic Inference
=============================

.. automodule:: dds_cloudapi_sdk.tasks.trex_generic
   :no-members:

Usage Pattern
-------------

First of all, make sure you have installed this SDK by `pip`::

    pip install dds-cloudapi-sdk

Then trigger the algorithm by TRexGenericInfer class::

    from dds_cloudapi_sdk import Config
    from dds_cloudapi_sdk import Client
    from dds_cloudapi_sdk import BatchRectPrompt
    from dds_cloudapi_sdk import TRexGenericInfer

    # Step 1: initialize the config
    token = "Your API token here"
    config = Config(token)

    # Step 2: initialize the client
    client = Client(config)

    # Step 3: run the task by TRexGenericInfer class

    image_url = "https://dev.deepdataspace.com/static/04_a.ae28c1d6.jpg"
    # if you are processing local image file, upload them to DDS server to get the image url
    # image_url = client.upload_file("/path/to/your/infer/image.png")

    # the generic inference supports prompts from multiple images,
    # but the prompts must be the same type, e.g. all point prompts or all rect prompts
    batch_prompts = [
        BatchRectPrompt(
            image=image_url,
            rects=[[475.18413597733706, 550.1983002832861, 548.1019830028329, 599.915014164306]]
        )
    ]

    task = TRexGenericInfer(
        image_url=image_url,
        batch_prompts=batch_prompts
    )

    client.run_task(task)
    for obj in task.result.objects:
        print(obj.score)
        print(obj.bbox)
        break


API Reference
-------------

.. autoclass:: TRexGenericInfer
   :members: result
   :exclude-members: __init__,format_result
   :inherited-members:

.. autoclass:: dds_cloudapi_sdk.tasks.prompt.BatchPointPrompt
   :members:
   :exclude-members: model_computed_fields,model_config,model_fields

.. autoclass:: dds_cloudapi_sdk.tasks.prompt.BatchRectPrompt
   :members:
   :exclude-members: model_computed_fields,model_config,model_fields

.. autoclass:: TaskResult
   :members:
   :exclude-members: model_computed_fields,model_config,model_fields

.. autoclass:: TRexObject
   :members:
   :exclude-members: model_computed_fields,model_config,model_fields
