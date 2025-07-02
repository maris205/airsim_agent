.. currentmodule:: dds_cloudapi_sdk.tasks.trex_embd_customize

Trex - Embedding Customization
==============================

.. automodule:: dds_cloudapi_sdk.tasks.trex_embd_customize
   :no-members:

Usage Pattern
-------------

First of all, make sure you have installed this SDK by `pip`::

    pip install dds-cloudapi-sdk

Then trigger the customization algorithm through TRexEmbdCustomize::

    from dds_cloudapi_sdk import Config
    from dds_cloudapi_sdk import Client
    from dds_cloudapi_sdk import BatchRectPrompt
    from dds_cloudapi_sdk import TRexEmbdCustomize

    # Step 1: initialize the config
    token = "Your API token here"
    config = Config(token)

    # Step 2: initialize the client
    client = Client(config)

    # Step 3: run the task by TRexEmbdCustomize class
    image_url = "https://dev.deepdataspace.com/static/04_a.ae28c1d6.jpg"
    # if you are processing local image file, upload them to DDS server to get the image url
    # image_url = client.upload_file("/path/to/your/infer/image.png")

    batch_prompts = [
        BatchRectPrompt(
            image=image_url,
            rects=[[475.18413597733706, 550.1983002832861, 548.1019830028329, 599.915014164306]]
        )
    ]
    task = TRexEmbdCustomize(
        batch_prompts=batch_prompts
    )

    client.run_task(task)
    embd_url = task.result.embd
    print(embd_url)


API Reference
-------------

.. autoclass:: TRexEmbdCustomize
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
