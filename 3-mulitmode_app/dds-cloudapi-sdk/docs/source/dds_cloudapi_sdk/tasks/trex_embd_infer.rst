.. currentmodule:: dds_cloudapi_sdk.tasks.trex_embd_infer

Trex - Embedding Inference
==============================

.. automodule:: dds_cloudapi_sdk.tasks.trex_embd_infer
   :no-members:

Usage Pattern
-------------

First of all, make sure you have installed this SDK by `pip`::

    pip install dds-cloudapi-sdk

Then trigger the customization algorithm by TRexEmbdInfer::

    from dds_cloudapi_sdk import Client
    from dds_cloudapi_sdk import Config
    from dds_cloudapi_sdk import BatchEmbdInfer
    from dds_cloudapi_sdk import BatchEmbdPrompt
    from dds_cloudapi_sdk import TRexEmbdInfer

    token = "You API token here"
    config = Config(token)
    client = Client(config)

    image_url = "https://dev.deepdataspace.com/static/04_a.ae28c1d6.jpg"
    embd_url = "https://url/to/your/embd/url.file"  # the embd url trained from TRexEmbdCustomize Task

    infer_1 = BatchEmbdInfer(
        image=image_url,
        prompts=[
            BatchEmbdPrompt(embd=embd_url, category_id=1)
        ]
    )
    task = TRexEmbdInfer([infer_1])
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

.. autoclass:: TRexEmbdInfer
   :members: result
   :exclude-members: __init__,format_result
   :inherited-members:

.. autoclass:: dds_cloudapi_sdk.tasks.prompt.BatchEmbdInfer
   :members:
   :exclude-members: model_computed_fields,model_config,model_fields

.. autoclass:: dds_cloudapi_sdk.tasks.prompt.BatchEmbdPrompt
   :members:
   :exclude-members: model_computed_fields,model_config,model_fields

.. autoclass:: TaskResult
   :members:
   :exclude-members: model_computed_fields,model_config,model_fields

.. autoclass:: TRexObject
   :members:
   :exclude-members: model_computed_fields,model_config,model_fields
