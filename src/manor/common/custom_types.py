from typing import Annotated, Literal

import numpy as np
import numpy.typing as npt

# Path types
FileName = str
FilePath = str
DirName = str
DirPath = str

# Numpy types
f64 = np.float64
i64 = np.int64
ui8 = np.uint8
NpArr = npt.NDArray
NpArrf64 = npt.NDArray[f64]

# Numpy types.
NpVectorNf64 = Annotated[NpArrf64, Literal["N"]]
NpVector1f64 = Annotated[NpArrf64, Literal["1"]]
NpVector2f64 = Annotated[NpArrf64, Literal["2"]]
NpVector3f64 = Annotated[NpArrf64, Literal["3"]]
NpVector4f64 = Annotated[NpArrf64, Literal["4"]]

# 2D matrix types
NpMatrixNMf64 = Annotated[NpArrf64, Literal["N", "M"]]
NpMatrixN3f64 = Annotated[NpArrf64, Literal["N", "3"]]
NpMatrixN4f64 = Annotated[NpArrf64, Literal["N", "4"]]

# Time stuff.
TimesVector = NpVectorNf64  # Time in seconds

# Control.
PositionsVector = NpVectorNf64
VelocitiesVector = NpVectorNf64
StateVector = NpVectorNf64
GainsVector = NpVectorNf64
ControlSignalVector = NpVectorNf64

JointPositionsVector = NpVectorNf64
JointVelocitiesVector = NpVectorNf64
JointStateVector = NpVectorNf64

GripperPositionsVector = NpVectorNf64
GripperVelocitiesVector = NpVectorNf64
