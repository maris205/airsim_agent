dds-cloudapi-sdk
============================================

The dds-cloudapi-sdk is a Python package designed to simplify interactions with the DDS Cloud API. It features:

- **Straightforward** APIs
- **Unified** interfaces
- **Handy** utilities

Installation
------------
You can get the SDK library directly from PyPi::

    pip install dds-cloudapi-sdk

Quick Start
-----------
Below is a straightforward example for the popular IVP algorithm::

    # 1. Initialize the client with your API token.
    from dds_cloudapi_sdk import Config
    from dds_cloudapi_sdk import Client

    token = "Your API Token Here"
    config = Config(token)
    client = Client(config)

    # 2. Upload local image to the server and get the URL.
    infer_image_url = "https://dev.deepdataspace.com/static/04_a.ae28c1d6.jpg"
    # infer_image_url = client.upload_file("path/to/infer/image.jpg")  # you can also upload local file for processing
    prompt_image_url = infer_image_url  # use the same image for prompt

    # 3. Create a task with proper parameters.
    from dds_cloudapi_sdk.tasks import IVPTask
    from dds_cloudapi_sdk.tasks import RectPrompt
    from dds_cloudapi_sdk.tasks import LabelTypes

    task = IVPTask(
        prompt_image_url=prompt_image_url,
        prompts=[RectPrompt(rect=[475.18, 550.20, 548.10, 599.92], is_positive=True)],
        infer_image_url=infer_image_url,
        infer_label_types=[LabelTypes.BBox, LabelTypes.Mask],  # infer both bbox and mask
    )

    # 4. Run the task and get the result.
    client.run_task(task)

    # 5. Parse the result.
    from dds_cloudapi_sdk.tasks.ivp import TaskResult

    result: TaskResult = task.result

    mask_url = result.mask_url  # the image url with all masks drawn on
    objects = result.objects  # the list of detected objects
    for idx, obj in enumerate(objects):
        # get the detection score
        print(obj.score)  # 0.42

        # get the detection bbox
        print(obj.bbox)  # [635.0, 458.0, 704.0, 508.0]

        # get the detection mask, it's of RLE format
        print(obj.mask.counts)  # ]o`f08fa14M3L2O2M2O1O1O1O1N2O1N2O1N2N3M2O3L3M3N2M2N3N1N2O...

        # convert the RLE format to RGBA image
        mask_image = task.rle2rgba(obj.mask)
        print(mask_image.size)  # (1600, 1170)

        # save the image to file
        mask_image.save(f"data/mask_{idx}.png")


Documentation
-------------
As illustrated above, executing an algorithm involves three key steps:

- **1. Config**: Initializing a configuration object with your API token.
- **2. Client**: Setting up a client object with the previously created config.
- **3. Tasks**: Executing a task through the client to obtain results.

The first two steps remain consistent across all algorithms, providing a unified interface for any task execution.

The rest of this document will briefly cover the Config and Client before diving into the various usage patterns of the algorithms (or task classes).

.. toctree::
   :maxdepth: 2

   dds_cloudapi_sdk/config
   dds_cloudapi_sdk/client

.. toctree::
   :maxdepth: 3

   dds_cloudapi_sdk/tasks
