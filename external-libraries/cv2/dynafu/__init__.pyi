import cv2
import cv2.kinfu
import cv2.typing
import typing


# Classes
class DynaFu:
    # Functions
    @classmethod
    def create(cls, _params: cv2.kinfu.Params) -> DynaFu: ...

    @typing.overload
    def render(self, image: cv2.typing.MatLike | None = ..., cameraPose: cv2.typing.Matx44f = ...) -> cv2.typing.MatLike: ...
    @typing.overload
    def render(self, image: cv2.UMat | None = ..., cameraPose: cv2.typing.Matx44f = ...) -> cv2.UMat: ...

    @typing.overload
    def getCloud(self, points: cv2.typing.MatLike | None = ..., normals: cv2.typing.MatLike | None = ...) -> tuple[cv2.typing.MatLike, cv2.typing.MatLike]: ...
    @typing.overload
    def getCloud(self, points: cv2.UMat | None = ..., normals: cv2.UMat | None = ...) -> tuple[cv2.UMat, cv2.UMat]: ...

    @typing.overload
    def getPoints(self, points: cv2.typing.MatLike | None = ...) -> cv2.typing.MatLike: ...
    @typing.overload
    def getPoints(self, points: cv2.UMat | None = ...) -> cv2.UMat: ...

    @typing.overload
    def getNormals(self, points: cv2.typing.MatLike, normals: cv2.typing.MatLike | None = ...) -> cv2.typing.MatLike: ...
    @typing.overload
    def getNormals(self, points: cv2.UMat, normals: cv2.UMat | None = ...) -> cv2.UMat: ...

    def reset(self) -> None: ...

    @typing.overload
    def update(self, depth: cv2.typing.MatLike) -> bool: ...
    @typing.overload
    def update(self, depth: cv2.UMat) -> bool: ...



