import cv2
import cv2.typing
import typing


# Classes
class WeChatQRCode:
    # Functions
    def __init__(self, detector_prototxt_path: str = ..., detector_caffe_model_path: str = ..., super_resolution_prototxt_path: str = ..., super_resolution_caffe_model_path: str = ...) -> None: ...

    @typing.overload
    def detectAndDecode(self, img: cv2.typing.MatLike, points: typing.Sequence[cv2.typing.MatLike] | None = ...) -> tuple[typing.Sequence[str], typing.Sequence[cv2.typing.MatLike]]: ...
    @typing.overload
    def detectAndDecode(self, img: cv2.UMat, points: typing.Sequence[cv2.UMat] | None = ...) -> tuple[typing.Sequence[str], typing.Sequence[cv2.UMat]]: ...

    def setScaleFactor(self, _scalingFactor: float) -> None: ...

    def getScaleFactor(self) -> float: ...



