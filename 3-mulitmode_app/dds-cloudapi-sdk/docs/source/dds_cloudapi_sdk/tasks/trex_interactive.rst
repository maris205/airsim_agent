.. currentmodule:: dds_cloudapi_sdk.tasks.trex_interactive

Trex - Interactive Inference
=============================

.. automodule:: dds_cloudapi_sdk.tasks.trex_interactive
   :no-members:

Usage Pattern
-------------

First of all, make sure you have installed this SDK by `pip`::

    pip install dds-cloudapi-sdk

Then trigger the algorithm through TRexInteractiveInfer class::

    from dds_cloudapi_sdk import Client
    from dds_cloudapi_sdk import Config
    from dds_cloudapi_sdk import BatchRectInfer
    from dds_cloudapi_sdk import BatchRectPrompt
    from dds_cloudapi_sdk import TRexInteractiveInfer

    # Step 1: initialize the config
    token = "Your API token here"
    config = Config(token)

    # Step 2: initialize the client
    client = Client(config)

    # Step 3: run the task by TRexInteractiveInfer class
    infer_image = "https://dev.deepdataspace.com/static/04_a.ae28c1d6.jpg"
    # if you are processing local image file, upload them to DDS server to get the image url
    # infer_image = client.upload_file("/path/to/your/infer/image.png")

    infer_1 = BatchRectInfer(
        image=infer_image,
        prompts=[
            BatchRectPrompt(category_id=1, rects=[[475.18413597733706, 550.1983002832861, 548.1019830028329, 599.915014164306]])
        ]
    )

    task = TRexInteractiveInfer([infer_1])  # the interactive infer task supports batch inference

    client.run_task(task)
    for image_objects in task.result.object_batches:
        for obj in image_objects:
            print(obj.score)
            print(obj.bbox)
            print(obj.category_id)
            break
        break


API Reference
-------------

.. autoclass:: TRexInteractiveInfer
   :members: result
   :exclude-members: __init__,format_result
   :inherited-members:

.. autoclass:: dds_cloudapi_sdk.tasks.prompt.BatchPointInfer
   :members:
   :exclude-members: model_computed_fields,model_config,model_fields

.. autoclass:: dds_cloudapi_sdk.tasks.prompt.BatchRectInfer
   :members:
   :exclude-members: model_computed_fields,model_config,model_fields

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
