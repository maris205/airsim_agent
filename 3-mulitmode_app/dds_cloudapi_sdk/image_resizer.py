import base64
from io import BytesIO
from typing import Tuple
from typing import Union

from PIL import Image


def _open_image(image_input: Union[str, bytes, BytesIO]) -> Image.Image:
    """Open image from file path, bytes or BytesIO object"""
    if isinstance(image_input, str):
        return Image.open(image_input)
    return Image.open(BytesIO(image_input) if isinstance(image_input, bytes) else image_input)


def _save_to_bytesio(img: Image.Image) -> BytesIO:
    """Save image to BytesIO object"""
    output = BytesIO()
    format = img.format or 'JPEG'

    if img.mode not in ('RGB', 'L'):
        img = img.convert('RGB')

    try:
        img.save(output, format=format)
    except OSError:
        output = BytesIO()
        img.save(output, format='PNG')

    output.seek(0)
    return output


def _get_original_bytesio(image_input: Union[str, bytes, BytesIO]) -> BytesIO:
    """Get BytesIO object from image input"""
    if isinstance(image_input, str):
        with open(image_input, 'rb') as f:
            return BytesIO(f.read())
    elif isinstance(image_input, bytes):
        return BytesIO(image_input)
    else:  # BytesIO
        image_input.seek(0)
        return BytesIO(image_input.getvalue())


def resize_image(
    image_input: Union[str, bytes, BytesIO],
    max_size: int = 1536
) -> Tuple[BytesIO, dict]:
    """
    Resize image so that the longest edge is no larger than max_size

    Returns:
        Tuple[BytesIO, dict]: (resized image data, {ratio: float, original_width: int, original_height: int})
        Returns original image and None if no resize needed
    """
    img = _open_image(image_input)
    width, height = img.size
    ratio = min(max_size / max(width, height), 1.0)

    if ratio >= 1.0:
        return _get_original_bytesio(image_input), None

    new_width = int(width * ratio)
    new_height = int(height * ratio)
    resized_img = img.resize((new_width, new_height), Image.Resampling.LANCZOS)

    return _save_to_bytesio(resized_img), {'ratio': ratio, 'original_width': width, 'original_height': height}


def resize_and_save_image(
    image_input: Union[str, bytes, BytesIO],
    output_path: str,
    max_size: int = 1536
) -> dict:
    """Resize image and save to output path"""
    resized_data, scale_info = resize_image(image_input, max_size)
    with open(output_path, 'wb') as f:
        f.write(resized_data.getvalue())
    return scale_info


def image_to_base64(image_input: Union[str, bytes, BytesIO]) -> str:
    """Convert image to base64 string with data URL format"""
    img = _open_image(image_input)
    format = img.format or 'JPEG'

    output = _get_original_bytesio(image_input)
    base64_str = base64.b64encode(output.getvalue()).decode('utf-8')

    return f"data:image/{format.lower()};base64,{base64_str}"
