import cv2
import cv2.typing
import typing


# Classes
class Tracker(cv2.Algorithm):
    # Functions
    @typing.overload
    def init(self, image: cv2.typing.MatLike, boundingBox: cv2.typing.Rect2d) -> bool: ...
    @typing.overload
    def init(self, image: cv2.UMat, boundingBox: cv2.typing.Rect2d) -> bool: ...

    @typing.overload
    def update(self, image: cv2.typing.MatLike) -> tuple[bool, cv2.typing.Rect2d]: ...
    @typing.overload
    def update(self, image: cv2.UMat) -> tuple[bool, cv2.typing.Rect2d]: ...


class MultiTracker(cv2.Algorithm):
    # Functions
    def __init__(self) -> None: ...

    @typing.overload
    def add(self, newTracker: Tracker, image: cv2.typing.MatLike, boundingBox: cv2.typing.Rect2d) -> bool: ...
    @typing.overload
    def add(self, newTracker: Tracker, image: cv2.UMat, boundingBox: cv2.typing.Rect2d) -> bool: ...

    @typing.overload
    def update(self, image: cv2.typing.MatLike) -> tuple[bool, typing.Sequence[cv2.typing.Rect2d]]: ...
    @typing.overload
    def update(self, image: cv2.UMat) -> tuple[bool, typing.Sequence[cv2.typing.Rect2d]]: ...

    def getObjects(self) -> typing.Sequence[cv2.typing.Rect2d]: ...

    @classmethod
    def create(cls) -> MultiTracker: ...


class TrackerMIL(Tracker):
    # Functions
    @classmethod
    def create(cls) -> TrackerMIL: ...


class TrackerBoosting(Tracker):
    # Functions
    @classmethod
    def create(cls) -> TrackerBoosting: ...


class TrackerMedianFlow(Tracker):
    # Functions
    @classmethod
    def create(cls) -> TrackerMedianFlow: ...


class TrackerTLD(Tracker):
    # Functions
    @classmethod
    def create(cls) -> TrackerTLD: ...


class TrackerKCF(Tracker):
    # Functions
    @classmethod
    def create(cls) -> TrackerKCF: ...


class TrackerMOSSE(Tracker):
    # Functions
    @classmethod
    def create(cls) -> TrackerMOSSE: ...


class TrackerCSRT(Tracker):
    # Functions
    @classmethod
    def create(cls) -> TrackerCSRT: ...

    @typing.overload
    def setInitialMask(self, mask: cv2.typing.MatLike) -> None: ...
    @typing.overload
    def setInitialMask(self, mask: cv2.UMat) -> None: ...



# Functions
def upgradeTrackingAPI(legacy_tracker: Tracker) -> Tracker: ...


