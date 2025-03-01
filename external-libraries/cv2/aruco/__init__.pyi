import cv2
import cv2.typing
import typing


# Enumerations
CORNER_REFINE_NONE: int
CORNER_REFINE_SUBPIX: int
CORNER_REFINE_CONTOUR: int
CORNER_REFINE_APRILTAG: int
CornerRefineMethod = int
"""One of [CORNER_REFINE_NONE, CORNER_REFINE_SUBPIX, CORNER_REFINE_CONTOUR, CORNER_REFINE_APRILTAG]"""

DICT_4X4_50: int
DICT_4X4_100: int
DICT_4X4_250: int
DICT_4X4_1000: int
DICT_5X5_50: int
DICT_5X5_100: int
DICT_5X5_250: int
DICT_5X5_1000: int
DICT_6X6_50: int
DICT_6X6_100: int
DICT_6X6_250: int
DICT_6X6_1000: int
DICT_7X7_50: int
DICT_7X7_100: int
DICT_7X7_250: int
DICT_7X7_1000: int
DICT_ARUCO_ORIGINAL: int
DICT_APRILTAG_16h5: int
DICT_APRILTAG_16H5: int
DICT_APRILTAG_25h9: int
DICT_APRILTAG_25H9: int
DICT_APRILTAG_36h10: int
DICT_APRILTAG_36H10: int
DICT_APRILTAG_36h11: int
DICT_APRILTAG_36H11: int
DICT_ARUCO_MIP_36h12: int
DICT_ARUCO_MIP_36H12: int
PredefinedDictionaryType = int
"""One of [DICT_4X4_50, DICT_4X4_100, DICT_4X4_250, DICT_4X4_1000, DICT_5X5_50, DICT_5X5_100, DICT_5X5_250, DICT_5X5_1000, DICT_6X6_50, DICT_6X6_100, DICT_6X6_250, DICT_6X6_1000, DICT_7X7_50, DICT_7X7_100, DICT_7X7_250, DICT_7X7_1000, DICT_ARUCO_ORIGINAL, DICT_APRILTAG_16h5, DICT_APRILTAG_16H5, DICT_APRILTAG_25h9, DICT_APRILTAG_25H9, DICT_APRILTAG_36h10, DICT_APRILTAG_36H10, DICT_APRILTAG_36h11, DICT_APRILTAG_36H11, DICT_ARUCO_MIP_36h12, DICT_ARUCO_MIP_36H12]"""

ARUCO_CCW_CENTER: int
ARUCO_CW_TOP_LEFT_CORNER: int
PatternPositionType = int
"""One of [ARUCO_CCW_CENTER, ARUCO_CW_TOP_LEFT_CORNER]"""



# Classes
class Board:
    # Functions
    @typing.overload
    def __init__(self, objPoints: typing.Sequence[cv2.typing.MatLike], dictionary: Dictionary, ids: cv2.typing.MatLike) -> None: ...
    @typing.overload
    def __init__(self, objPoints: typing.Sequence[cv2.UMat], dictionary: Dictionary, ids: cv2.UMat) -> None: ...

    def getDictionary(self) -> Dictionary: ...

    def getObjPoints(self) -> typing.Sequence[typing.Sequence[cv2.typing.Point3f]]: ...

    def getIds(self) -> typing.Sequence[int]: ...

    def getRightBottomCorner(self) -> cv2.typing.Point3f: ...

    @typing.overload
    def matchImagePoints(self, detectedCorners: typing.Sequence[cv2.typing.MatLike], detectedIds: cv2.typing.MatLike, objPoints: cv2.typing.MatLike | None = ..., imgPoints: cv2.typing.MatLike | None = ...) -> tuple[cv2.typing.MatLike, cv2.typing.MatLike]: ...
    @typing.overload
    def matchImagePoints(self, detectedCorners: typing.Sequence[cv2.UMat], detectedIds: cv2.UMat, objPoints: cv2.UMat | None = ..., imgPoints: cv2.UMat | None = ...) -> tuple[cv2.UMat, cv2.UMat]: ...

    @typing.overload
    def generateImage(self, outSize: cv2.typing.Size, img: cv2.typing.MatLike | None = ..., marginSize: int = ..., borderBits: int = ...) -> cv2.typing.MatLike: ...
    @typing.overload
    def generateImage(self, outSize: cv2.typing.Size, img: cv2.UMat | None = ..., marginSize: int = ..., borderBits: int = ...) -> cv2.UMat: ...


class DetectorParameters:
    adaptiveThreshWinSizeMin: int
    adaptiveThreshWinSizeMax: int
    adaptiveThreshWinSizeStep: int
    adaptiveThreshConstant: float
    minMarkerPerimeterRate: float
    maxMarkerPerimeterRate: float
    polygonalApproxAccuracyRate: float
    minCornerDistanceRate: float
    minDistanceToBorder: int
    minMarkerDistanceRate: float
    cornerRefinementMethod: int
    cornerRefinementWinSize: int
    cornerRefinementMaxIterations: int
    cornerRefinementMinAccuracy: float
    markerBorderBits: int
    perspectiveRemovePixelPerCell: int
    perspectiveRemoveIgnoredMarginPerCell: float
    maxErroneousBitsInBorderRate: float
    minOtsuStdDev: float
    errorCorrectionRate: float
    aprilTagQuadDecimate: float
    aprilTagQuadSigma: float
    aprilTagMinClusterPixels: int
    aprilTagMaxNmaxima: int
    aprilTagCriticalRad: float
    aprilTagMaxLineFitMse: float
    aprilTagMinWhiteBlackDiff: int
    aprilTagDeglitch: int
    detectInvertedMarker: bool
    useAruco3Detection: bool
    minSideLengthCanonicalImg: int
    minMarkerLengthRatioOriginalImg: float

    # Functions
    def __init__(self) -> None: ...

    def readDetectorParameters(self, fn: cv2.FileNode) -> bool: ...

    def writeDetectorParameters(self, fs: cv2.FileStorage, name: str = ...) -> bool: ...


class RefineParameters:
    minRepDistance: float
    errorCorrectionRate: float
    checkAllOrders: bool

    # Functions
    def __init__(self, minRepDistance: float = ..., errorCorrectionRate: float = ..., checkAllOrders: bool = ...) -> None: ...

    def readRefineParameters(self, fn: cv2.FileNode) -> bool: ...

    def writeRefineParameters(self, fs: cv2.FileStorage, name: str = ...) -> bool: ...


class Dictionary:
    bytesList: cv2.typing.MatLike
    markerSize: int
    maxCorrectionBits: int

    # Functions
    @typing.overload
    def __init__(self) -> None: ...
    @typing.overload
    def __init__(self, bytesList: cv2.typing.MatLike, _markerSize: int, maxcorr: int = ...) -> None: ...

    def readDictionary(self, fn: cv2.FileNode) -> bool: ...

    def writeDictionary(self, fs: cv2.FileStorage, name: str = ...) -> None: ...

    def identify(self, onlyBits: cv2.typing.MatLike, maxCorrectionRate: float) -> tuple[bool, int, int]: ...

    @typing.overload
    def getDistanceToId(self, bits: cv2.typing.MatLike, id: int, allRotations: bool = ...) -> int: ...
    @typing.overload
    def getDistanceToId(self, bits: cv2.UMat, id: int, allRotations: bool = ...) -> int: ...

    @typing.overload
    def generateImageMarker(self, id: int, sidePixels: int, _img: cv2.typing.MatLike | None = ..., borderBits: int = ...) -> cv2.typing.MatLike: ...
    @typing.overload
    def generateImageMarker(self, id: int, sidePixels: int, _img: cv2.UMat | None = ..., borderBits: int = ...) -> cv2.UMat: ...

    @staticmethod
    def getByteListFromBits(bits: cv2.typing.MatLike) -> cv2.typing.MatLike: ...

    @staticmethod
    def getBitsFromByteList(byteList: cv2.typing.MatLike, markerSize: int) -> cv2.typing.MatLike: ...


class CharucoParameters:
    cameraMatrix: cv2.typing.MatLike
    distCoeffs: cv2.typing.MatLike
    minMarkers: int
    tryRefineMarkers: bool

    # Functions
    def __init__(self) -> None: ...


class EstimateParameters:
    pattern: PatternPositionType
    useExtrinsicGuess: bool
    solvePnPMethod: int

    # Functions
    def __init__(self) -> None: ...


class GridBoard(Board):
    # Functions
    @typing.overload
    def __init__(self, size: cv2.typing.Size, markerLength: float, markerSeparation: float, dictionary: Dictionary, ids: cv2.typing.MatLike | None = ...) -> None: ...
    @typing.overload
    def __init__(self, size: cv2.typing.Size, markerLength: float, markerSeparation: float, dictionary: Dictionary, ids: cv2.UMat | None = ...) -> None: ...

    def getGridSize(self) -> cv2.typing.Size: ...

    def getMarkerLength(self) -> float: ...

    def getMarkerSeparation(self) -> float: ...


class CharucoBoard(Board):
    # Functions
    @typing.overload
    def __init__(self, size: cv2.typing.Size, squareLength: float, markerLength: float, dictionary: Dictionary, ids: cv2.typing.MatLike | None = ...) -> None: ...
    @typing.overload
    def __init__(self, size: cv2.typing.Size, squareLength: float, markerLength: float, dictionary: Dictionary, ids: cv2.UMat | None = ...) -> None: ...

    def setLegacyPattern(self, legacyPattern: bool) -> None: ...

    def getLegacyPattern(self) -> bool: ...

    def getChessboardSize(self) -> cv2.typing.Size: ...

    def getSquareLength(self) -> float: ...

    def getMarkerLength(self) -> float: ...

    def getChessboardCorners(self) -> typing.Sequence[cv2.typing.Point3f]: ...

    @typing.overload
    def checkCharucoCornersCollinear(self, charucoIds: cv2.typing.MatLike) -> bool: ...
    @typing.overload
    def checkCharucoCornersCollinear(self, charucoIds: cv2.UMat) -> bool: ...


class ArucoDetector(cv2.Algorithm):
    # Functions
    def __init__(self, dictionary: Dictionary = ..., detectorParams: DetectorParameters = ..., refineParams: RefineParameters = ...) -> None: ...

    @typing.overload
    def detectMarkers(self, image: cv2.typing.MatLike, corners: typing.Sequence[cv2.typing.MatLike] | None = ..., ids: cv2.typing.MatLike | None = ..., rejectedImgPoints: typing.Sequence[cv2.typing.MatLike] | None = ...) -> tuple[typing.Sequence[cv2.typing.MatLike], cv2.typing.MatLike, typing.Sequence[cv2.typing.MatLike]]: ...
    @typing.overload
    def detectMarkers(self, image: cv2.UMat, corners: typing.Sequence[cv2.UMat] | None = ..., ids: cv2.UMat | None = ..., rejectedImgPoints: typing.Sequence[cv2.UMat] | None = ...) -> tuple[typing.Sequence[cv2.UMat], cv2.UMat, typing.Sequence[cv2.UMat]]: ...

    @typing.overload
    def refineDetectedMarkers(self, image: cv2.typing.MatLike, board: Board, detectedCorners: typing.Sequence[cv2.typing.MatLike], detectedIds: cv2.typing.MatLike, rejectedCorners: typing.Sequence[cv2.typing.MatLike], cameraMatrix: cv2.typing.MatLike | None = ..., distCoeffs: cv2.typing.MatLike | None = ..., recoveredIdxs: cv2.typing.MatLike | None = ...) -> tuple[typing.Sequence[cv2.typing.MatLike], cv2.typing.MatLike, typing.Sequence[cv2.typing.MatLike], cv2.typing.MatLike]: ...
    @typing.overload
    def refineDetectedMarkers(self, image: cv2.UMat, board: Board, detectedCorners: typing.Sequence[cv2.UMat], detectedIds: cv2.UMat, rejectedCorners: typing.Sequence[cv2.UMat], cameraMatrix: cv2.UMat | None = ..., distCoeffs: cv2.UMat | None = ..., recoveredIdxs: cv2.UMat | None = ...) -> tuple[typing.Sequence[cv2.UMat], cv2.UMat, typing.Sequence[cv2.UMat], cv2.UMat]: ...

    def getDictionary(self) -> Dictionary: ...

    def setDictionary(self, dictionary: Dictionary) -> None: ...

    def getDetectorParameters(self) -> DetectorParameters: ...

    def setDetectorParameters(self, detectorParameters: DetectorParameters) -> None: ...

    def getRefineParameters(self) -> RefineParameters: ...

    def setRefineParameters(self, refineParameters: RefineParameters) -> None: ...

    def write(self, fs: cv2.FileStorage, name: str) -> None: ...

    def read(self, fn: cv2.FileNode) -> None: ...


class CharucoDetector(cv2.Algorithm):
    # Functions
    def __init__(self, board: CharucoBoard, charucoParams: CharucoParameters = ..., detectorParams: DetectorParameters = ..., refineParams: RefineParameters = ...) -> None: ...

    def getBoard(self) -> CharucoBoard: ...

    def setBoard(self, board: CharucoBoard) -> None: ...

    def getCharucoParameters(self) -> CharucoParameters: ...

    def setCharucoParameters(self, charucoParameters: CharucoParameters) -> None: ...

    def getDetectorParameters(self) -> DetectorParameters: ...

    def setDetectorParameters(self, detectorParameters: DetectorParameters) -> None: ...

    def getRefineParameters(self) -> RefineParameters: ...

    def setRefineParameters(self, refineParameters: RefineParameters) -> None: ...

    @typing.overload
    def detectBoard(self, image: cv2.typing.MatLike, charucoCorners: cv2.typing.MatLike | None = ..., charucoIds: cv2.typing.MatLike | None = ..., markerCorners: typing.Sequence[cv2.typing.MatLike] | None = ..., markerIds: cv2.typing.MatLike | None = ...) -> tuple[cv2.typing.MatLike, cv2.typing.MatLike, typing.Sequence[cv2.typing.MatLike], cv2.typing.MatLike]: ...
    @typing.overload
    def detectBoard(self, image: cv2.UMat, charucoCorners: cv2.UMat | None = ..., charucoIds: cv2.UMat | None = ..., markerCorners: typing.Sequence[cv2.UMat] | None = ..., markerIds: cv2.UMat | None = ...) -> tuple[cv2.UMat, cv2.UMat, typing.Sequence[cv2.UMat], cv2.UMat]: ...

    @typing.overload
    def detectDiamonds(self, image: cv2.typing.MatLike, diamondCorners: typing.Sequence[cv2.typing.MatLike] | None = ..., diamondIds: cv2.typing.MatLike | None = ..., markerCorners: typing.Sequence[cv2.typing.MatLike] | None = ..., markerIds: cv2.typing.MatLike | None = ...) -> tuple[typing.Sequence[cv2.typing.MatLike], cv2.typing.MatLike, typing.Sequence[cv2.typing.MatLike], cv2.typing.MatLike]: ...
    @typing.overload
    def detectDiamonds(self, image: cv2.UMat, diamondCorners: typing.Sequence[cv2.UMat] | None = ..., diamondIds: cv2.UMat | None = ..., markerCorners: typing.Sequence[cv2.UMat] | None = ..., markerIds: cv2.UMat | None = ...) -> tuple[typing.Sequence[cv2.UMat], cv2.UMat, typing.Sequence[cv2.UMat], cv2.UMat]: ...



# Functions
@typing.overload
def calibrateCameraAruco(corners: typing.Sequence[cv2.typing.MatLike], ids: cv2.typing.MatLike, counter: cv2.typing.MatLike, board: Board, imageSize: cv2.typing.Size, cameraMatrix: cv2.typing.MatLike, distCoeffs: cv2.typing.MatLike, rvecs: typing.Sequence[cv2.typing.MatLike] | None = ..., tvecs: typing.Sequence[cv2.typing.MatLike] | None = ..., flags: int = ..., criteria: cv2.typing.TermCriteria = ...) -> tuple[float, cv2.typing.MatLike, cv2.typing.MatLike, typing.Sequence[cv2.typing.MatLike], typing.Sequence[cv2.typing.MatLike]]: ...
@typing.overload
def calibrateCameraAruco(corners: typing.Sequence[cv2.UMat], ids: cv2.UMat, counter: cv2.UMat, board: Board, imageSize: cv2.typing.Size, cameraMatrix: cv2.UMat, distCoeffs: cv2.UMat, rvecs: typing.Sequence[cv2.UMat] | None = ..., tvecs: typing.Sequence[cv2.UMat] | None = ..., flags: int = ..., criteria: cv2.typing.TermCriteria = ...) -> tuple[float, cv2.UMat, cv2.UMat, typing.Sequence[cv2.UMat], typing.Sequence[cv2.UMat]]: ...

@typing.overload
def calibrateCameraArucoExtended(corners: typing.Sequence[cv2.typing.MatLike], ids: cv2.typing.MatLike, counter: cv2.typing.MatLike, board: Board, imageSize: cv2.typing.Size, cameraMatrix: cv2.typing.MatLike, distCoeffs: cv2.typing.MatLike, rvecs: typing.Sequence[cv2.typing.MatLike] | None = ..., tvecs: typing.Sequence[cv2.typing.MatLike] | None = ..., stdDeviationsIntrinsics: cv2.typing.MatLike | None = ..., stdDeviationsExtrinsics: cv2.typing.MatLike | None = ..., perViewErrors: cv2.typing.MatLike | None = ..., flags: int = ..., criteria: cv2.typing.TermCriteria = ...) -> tuple[float, cv2.typing.MatLike, cv2.typing.MatLike, typing.Sequence[cv2.typing.MatLike], typing.Sequence[cv2.typing.MatLike], cv2.typing.MatLike, cv2.typing.MatLike, cv2.typing.MatLike]: ...
@typing.overload
def calibrateCameraArucoExtended(corners: typing.Sequence[cv2.UMat], ids: cv2.UMat, counter: cv2.UMat, board: Board, imageSize: cv2.typing.Size, cameraMatrix: cv2.UMat, distCoeffs: cv2.UMat, rvecs: typing.Sequence[cv2.UMat] | None = ..., tvecs: typing.Sequence[cv2.UMat] | None = ..., stdDeviationsIntrinsics: cv2.UMat | None = ..., stdDeviationsExtrinsics: cv2.UMat | None = ..., perViewErrors: cv2.UMat | None = ..., flags: int = ..., criteria: cv2.typing.TermCriteria = ...) -> tuple[float, cv2.UMat, cv2.UMat, typing.Sequence[cv2.UMat], typing.Sequence[cv2.UMat], cv2.UMat, cv2.UMat, cv2.UMat]: ...

@typing.overload
def calibrateCameraCharuco(charucoCorners: typing.Sequence[cv2.typing.MatLike], charucoIds: typing.Sequence[cv2.typing.MatLike], board: CharucoBoard, imageSize: cv2.typing.Size, cameraMatrix: cv2.typing.MatLike, distCoeffs: cv2.typing.MatLike, rvecs: typing.Sequence[cv2.typing.MatLike] | None = ..., tvecs: typing.Sequence[cv2.typing.MatLike] | None = ..., flags: int = ..., criteria: cv2.typing.TermCriteria = ...) -> tuple[float, cv2.typing.MatLike, cv2.typing.MatLike, typing.Sequence[cv2.typing.MatLike], typing.Sequence[cv2.typing.MatLike]]: ...
@typing.overload
def calibrateCameraCharuco(charucoCorners: typing.Sequence[cv2.UMat], charucoIds: typing.Sequence[cv2.UMat], board: CharucoBoard, imageSize: cv2.typing.Size, cameraMatrix: cv2.UMat, distCoeffs: cv2.UMat, rvecs: typing.Sequence[cv2.UMat] | None = ..., tvecs: typing.Sequence[cv2.UMat] | None = ..., flags: int = ..., criteria: cv2.typing.TermCriteria = ...) -> tuple[float, cv2.UMat, cv2.UMat, typing.Sequence[cv2.UMat], typing.Sequence[cv2.UMat]]: ...

@typing.overload
def calibrateCameraCharucoExtended(charucoCorners: typing.Sequence[cv2.typing.MatLike], charucoIds: typing.Sequence[cv2.typing.MatLike], board: CharucoBoard, imageSize: cv2.typing.Size, cameraMatrix: cv2.typing.MatLike, distCoeffs: cv2.typing.MatLike, rvecs: typing.Sequence[cv2.typing.MatLike] | None = ..., tvecs: typing.Sequence[cv2.typing.MatLike] | None = ..., stdDeviationsIntrinsics: cv2.typing.MatLike | None = ..., stdDeviationsExtrinsics: cv2.typing.MatLike | None = ..., perViewErrors: cv2.typing.MatLike | None = ..., flags: int = ..., criteria: cv2.typing.TermCriteria = ...) -> tuple[float, cv2.typing.MatLike, cv2.typing.MatLike, typing.Sequence[cv2.typing.MatLike], typing.Sequence[cv2.typing.MatLike], cv2.typing.MatLike, cv2.typing.MatLike, cv2.typing.MatLike]: ...
@typing.overload
def calibrateCameraCharucoExtended(charucoCorners: typing.Sequence[cv2.UMat], charucoIds: typing.Sequence[cv2.UMat], board: CharucoBoard, imageSize: cv2.typing.Size, cameraMatrix: cv2.UMat, distCoeffs: cv2.UMat, rvecs: typing.Sequence[cv2.UMat] | None = ..., tvecs: typing.Sequence[cv2.UMat] | None = ..., stdDeviationsIntrinsics: cv2.UMat | None = ..., stdDeviationsExtrinsics: cv2.UMat | None = ..., perViewErrors: cv2.UMat | None = ..., flags: int = ..., criteria: cv2.typing.TermCriteria = ...) -> tuple[float, cv2.UMat, cv2.UMat, typing.Sequence[cv2.UMat], typing.Sequence[cv2.UMat], cv2.UMat, cv2.UMat, cv2.UMat]: ...

@typing.overload
def detectCharucoDiamond(image: cv2.typing.MatLike, markerCorners: typing.Sequence[cv2.typing.MatLike], markerIds: cv2.typing.MatLike, squareMarkerLengthRate: float, diamondCorners: typing.Sequence[cv2.typing.MatLike] | None = ..., diamondIds: cv2.typing.MatLike | None = ..., cameraMatrix: cv2.typing.MatLike | None = ..., distCoeffs: cv2.typing.MatLike | None = ..., dictionary: Dictionary = ...) -> tuple[typing.Sequence[cv2.typing.MatLike], cv2.typing.MatLike]: ...
@typing.overload
def detectCharucoDiamond(image: cv2.UMat, markerCorners: typing.Sequence[cv2.UMat], markerIds: cv2.UMat, squareMarkerLengthRate: float, diamondCorners: typing.Sequence[cv2.UMat] | None = ..., diamondIds: cv2.UMat | None = ..., cameraMatrix: cv2.UMat | None = ..., distCoeffs: cv2.UMat | None = ..., dictionary: Dictionary = ...) -> tuple[typing.Sequence[cv2.UMat], cv2.UMat]: ...

@typing.overload
def detectMarkers(image: cv2.typing.MatLike, dictionary: Dictionary, corners: typing.Sequence[cv2.typing.MatLike] | None = ..., ids: cv2.typing.MatLike | None = ..., parameters: DetectorParameters = ..., rejectedImgPoints: typing.Sequence[cv2.typing.MatLike] | None = ...) -> tuple[typing.Sequence[cv2.typing.MatLike], cv2.typing.MatLike, typing.Sequence[cv2.typing.MatLike]]: ...
@typing.overload
def detectMarkers(image: cv2.UMat, dictionary: Dictionary, corners: typing.Sequence[cv2.UMat] | None = ..., ids: cv2.UMat | None = ..., parameters: DetectorParameters = ..., rejectedImgPoints: typing.Sequence[cv2.UMat] | None = ...) -> tuple[typing.Sequence[cv2.UMat], cv2.UMat, typing.Sequence[cv2.UMat]]: ...

@typing.overload
def drawCharucoDiamond(dictionary: Dictionary, ids: cv2.typing.Vec4i, squareLength: int, markerLength: int, img: cv2.typing.MatLike | None = ..., marginSize: int = ..., borderBits: int = ...) -> cv2.typing.MatLike: ...
@typing.overload
def drawCharucoDiamond(dictionary: Dictionary, ids: cv2.typing.Vec4i, squareLength: int, markerLength: int, img: cv2.UMat | None = ..., marginSize: int = ..., borderBits: int = ...) -> cv2.UMat: ...

@typing.overload
def drawDetectedCornersCharuco(image: cv2.typing.MatLike, charucoCorners: cv2.typing.MatLike, charucoIds: cv2.typing.MatLike | None = ..., cornerColor: cv2.typing.Scalar = ...) -> cv2.typing.MatLike: ...
@typing.overload
def drawDetectedCornersCharuco(image: cv2.UMat, charucoCorners: cv2.UMat, charucoIds: cv2.UMat | None = ..., cornerColor: cv2.typing.Scalar = ...) -> cv2.UMat: ...

@typing.overload
def drawDetectedDiamonds(image: cv2.typing.MatLike, diamondCorners: typing.Sequence[cv2.typing.MatLike], diamondIds: cv2.typing.MatLike | None = ..., borderColor: cv2.typing.Scalar = ...) -> cv2.typing.MatLike: ...
@typing.overload
def drawDetectedDiamonds(image: cv2.UMat, diamondCorners: typing.Sequence[cv2.UMat], diamondIds: cv2.UMat | None = ..., borderColor: cv2.typing.Scalar = ...) -> cv2.UMat: ...

@typing.overload
def drawDetectedMarkers(image: cv2.typing.MatLike, corners: typing.Sequence[cv2.typing.MatLike], ids: cv2.typing.MatLike | None = ..., borderColor: cv2.typing.Scalar = ...) -> cv2.typing.MatLike: ...
@typing.overload
def drawDetectedMarkers(image: cv2.UMat, corners: typing.Sequence[cv2.UMat], ids: cv2.UMat | None = ..., borderColor: cv2.typing.Scalar = ...) -> cv2.UMat: ...

@typing.overload
def drawPlanarBoard(board: Board, outSize: cv2.typing.Size, marginSize: int, borderBits: int, img: cv2.typing.MatLike | None = ...) -> cv2.typing.MatLike: ...
@typing.overload
def drawPlanarBoard(board: Board, outSize: cv2.typing.Size, marginSize: int, borderBits: int, img: cv2.UMat | None = ...) -> cv2.UMat: ...

@typing.overload
def estimatePoseBoard(corners: typing.Sequence[cv2.typing.MatLike], ids: cv2.typing.MatLike, board: Board, cameraMatrix: cv2.typing.MatLike, distCoeffs: cv2.typing.MatLike, rvec: cv2.typing.MatLike, tvec: cv2.typing.MatLike, useExtrinsicGuess: bool = ...) -> tuple[int, cv2.typing.MatLike, cv2.typing.MatLike]: ...
@typing.overload
def estimatePoseBoard(corners: typing.Sequence[cv2.UMat], ids: cv2.UMat, board: Board, cameraMatrix: cv2.UMat, distCoeffs: cv2.UMat, rvec: cv2.UMat, tvec: cv2.UMat, useExtrinsicGuess: bool = ...) -> tuple[int, cv2.UMat, cv2.UMat]: ...

@typing.overload
def estimatePoseCharucoBoard(charucoCorners: cv2.typing.MatLike, charucoIds: cv2.typing.MatLike, board: CharucoBoard, cameraMatrix: cv2.typing.MatLike, distCoeffs: cv2.typing.MatLike, rvec: cv2.typing.MatLike, tvec: cv2.typing.MatLike, useExtrinsicGuess: bool = ...) -> tuple[bool, cv2.typing.MatLike, cv2.typing.MatLike]: ...
@typing.overload
def estimatePoseCharucoBoard(charucoCorners: cv2.UMat, charucoIds: cv2.UMat, board: CharucoBoard, cameraMatrix: cv2.UMat, distCoeffs: cv2.UMat, rvec: cv2.UMat, tvec: cv2.UMat, useExtrinsicGuess: bool = ...) -> tuple[bool, cv2.UMat, cv2.UMat]: ...

@typing.overload
def estimatePoseSingleMarkers(corners: typing.Sequence[cv2.typing.MatLike], markerLength: float, cameraMatrix: cv2.typing.MatLike, distCoeffs: cv2.typing.MatLike, rvecs: cv2.typing.MatLike | None = ..., tvecs: cv2.typing.MatLike | None = ..., objPoints: cv2.typing.MatLike | None = ..., estimateParameters: EstimateParameters = ...) -> tuple[cv2.typing.MatLike, cv2.typing.MatLike, cv2.typing.MatLike]: ...
@typing.overload
def estimatePoseSingleMarkers(corners: typing.Sequence[cv2.UMat], markerLength: float, cameraMatrix: cv2.UMat, distCoeffs: cv2.UMat, rvecs: cv2.UMat | None = ..., tvecs: cv2.UMat | None = ..., objPoints: cv2.UMat | None = ..., estimateParameters: EstimateParameters = ...) -> tuple[cv2.UMat, cv2.UMat, cv2.UMat]: ...

def extendDictionary(nMarkers: int, markerSize: int, baseDictionary: Dictionary = ..., randomSeed: int = ...) -> Dictionary: ...

@typing.overload
def generateImageMarker(dictionary: Dictionary, id: int, sidePixels: int, img: cv2.typing.MatLike | None = ..., borderBits: int = ...) -> cv2.typing.MatLike: ...
@typing.overload
def generateImageMarker(dictionary: Dictionary, id: int, sidePixels: int, img: cv2.UMat | None = ..., borderBits: int = ...) -> cv2.UMat: ...

@typing.overload
def getBoardObjectAndImagePoints(board: Board, detectedCorners: typing.Sequence[cv2.typing.MatLike], detectedIds: cv2.typing.MatLike, objPoints: cv2.typing.MatLike | None = ..., imgPoints: cv2.typing.MatLike | None = ...) -> tuple[cv2.typing.MatLike, cv2.typing.MatLike]: ...
@typing.overload
def getBoardObjectAndImagePoints(board: Board, detectedCorners: typing.Sequence[cv2.UMat], detectedIds: cv2.UMat, objPoints: cv2.UMat | None = ..., imgPoints: cv2.UMat | None = ...) -> tuple[cv2.UMat, cv2.UMat]: ...

def getPredefinedDictionary(dict: int) -> Dictionary: ...

@typing.overload
def interpolateCornersCharuco(markerCorners: typing.Sequence[cv2.typing.MatLike], markerIds: cv2.typing.MatLike, image: cv2.typing.MatLike, board: CharucoBoard, charucoCorners: cv2.typing.MatLike | None = ..., charucoIds: cv2.typing.MatLike | None = ..., cameraMatrix: cv2.typing.MatLike | None = ..., distCoeffs: cv2.typing.MatLike | None = ..., minMarkers: int = ...) -> tuple[int, cv2.typing.MatLike, cv2.typing.MatLike]: ...
@typing.overload
def interpolateCornersCharuco(markerCorners: typing.Sequence[cv2.UMat], markerIds: cv2.UMat, image: cv2.UMat, board: CharucoBoard, charucoCorners: cv2.UMat | None = ..., charucoIds: cv2.UMat | None = ..., cameraMatrix: cv2.UMat | None = ..., distCoeffs: cv2.UMat | None = ..., minMarkers: int = ...) -> tuple[int, cv2.UMat, cv2.UMat]: ...

@typing.overload
def refineDetectedMarkers(image: cv2.typing.MatLike, board: Board, detectedCorners: typing.Sequence[cv2.typing.MatLike], detectedIds: cv2.typing.MatLike, rejectedCorners: typing.Sequence[cv2.typing.MatLike], cameraMatrix: cv2.typing.MatLike | None = ..., distCoeffs: cv2.typing.MatLike | None = ..., minRepDistance: float = ..., errorCorrectionRate: float = ..., checkAllOrders: bool = ..., recoveredIdxs: cv2.typing.MatLike | None = ..., parameters: DetectorParameters = ...) -> tuple[typing.Sequence[cv2.typing.MatLike], cv2.typing.MatLike, typing.Sequence[cv2.typing.MatLike], cv2.typing.MatLike]: ...
@typing.overload
def refineDetectedMarkers(image: cv2.UMat, board: Board, detectedCorners: typing.Sequence[cv2.UMat], detectedIds: cv2.UMat, rejectedCorners: typing.Sequence[cv2.UMat], cameraMatrix: cv2.UMat | None = ..., distCoeffs: cv2.UMat | None = ..., minRepDistance: float = ..., errorCorrectionRate: float = ..., checkAllOrders: bool = ..., recoveredIdxs: cv2.UMat | None = ..., parameters: DetectorParameters = ...) -> tuple[typing.Sequence[cv2.UMat], cv2.UMat, typing.Sequence[cv2.UMat], cv2.UMat]: ...

@typing.overload
def testCharucoCornersCollinear(board: CharucoBoard, charucoIds: cv2.typing.MatLike) -> bool: ...
@typing.overload
def testCharucoCornersCollinear(board: CharucoBoard, charucoIds: cv2.UMat) -> bool: ...


