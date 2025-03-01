import cv2.typing
import typing


# Enumerations
TraitAs_TENSOR: int
TRAIT_AS_TENSOR: int
TraitAs_IMAGE: int
TRAIT_AS_IMAGE: int
TraitAs = int
"""One of [TraitAs_TENSOR, TRAIT_AS_TENSOR, TraitAs_IMAGE, TRAIT_AS_IMAGE]"""



# Classes
class PyParams:
    # Functions
    @typing.overload
    def __init__(self) -> None: ...
    @typing.overload
    def __init__(self, tag: str, model_path: str) -> None: ...

    def cfgMeanStd(self, layer_name: str, m: cv2.typing.Scalar, s: cv2.typing.Scalar) -> PyParams: ...

    def cfgNormalize(self, layer_name: str, flag: bool) -> PyParams: ...



# Functions
def params(tag: str, model_path: str) -> PyParams: ...


