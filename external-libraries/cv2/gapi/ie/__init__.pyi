import cv2.typing
import typing


# Enumerations
TraitAs_TENSOR: int
TRAIT_AS_TENSOR: int
TraitAs_IMAGE: int
TRAIT_AS_IMAGE: int
TraitAs = int
"""One of [TraitAs_TENSOR, TRAIT_AS_TENSOR, TraitAs_IMAGE, TRAIT_AS_IMAGE]"""

Sync: int
SYNC: int
Async: int
ASYNC: int
InferMode = int
"""One of [Sync, SYNC, Async, ASYNC]"""



# Classes
class PyParams:
    # Functions
    @typing.overload
    def __init__(self) -> None: ...
    @typing.overload
    def __init__(self, tag: str, model: str, weights: str, device: str) -> None: ...
    @typing.overload
    def __init__(self, tag: str, model: str, device: str) -> None: ...

    def constInput(self, layer_name: str, data: cv2.typing.MatLike, hint: TraitAs = ...) -> PyParams: ...

    def cfgNumRequests(self, nireq: int) -> PyParams: ...

    def cfgBatchSize(self, size: int) -> PyParams: ...



# Functions
@typing.overload
def params(tag: str, model: str, weights: str, device: str) -> PyParams: ...
@typing.overload
def params(tag: str, model: str, device: str) -> PyParams: ...


