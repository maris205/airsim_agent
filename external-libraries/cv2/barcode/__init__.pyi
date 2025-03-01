import cv2
import cv2.typing
import typing


# Classes
class BarcodeDetector(cv2.GraphicalCodeDetector):
    # Functions
    @typing.overload
    def __init__(self) -> None: ...
    @typing.overload
    def __init__(self, prototxt_path: str, model_path: str) -> None: ...

    @typing.overload
    def decodeWithType(self, img: cv2.typing.MatLike, points: cv2.typing.MatLike) -> tuple[bool, typing.Sequence[str], typing.Sequence[str]]: ...
    @typing.overload
    def decodeWithType(self, img: cv2.UMat, points: cv2.UMat) -> tuple[bool, typing.Sequence[str], typing.Sequence[str]]: ...

    @typing.overload
    def detectAndDecodeWithType(self, img: cv2.typing.MatLike, points: cv2.typing.MatLike | None = ...) -> tuple[bool, typing.Sequence[str], typing.Sequence[str], cv2.typing.MatLike]: ...
    @typing.overload
    def detectAndDecodeWithType(self, img: cv2.UMat, points: cv2.UMat | None = ...) -> tuple[bool, typing.Sequence[str], typing.Sequence[str], cv2.UMat]: ...



