from typing import Annotated, Literal

import numpy as np
import numpy.typing as npt

# Path types
type FileName = str
type FilePath = str
type DirName = str
type DirPath = str

# Numpy scalar + array types
type f64 = np.float64
type i64 = np.int64
type ui8 = np.uint8
type NpArr = npt.NDArray
type NpArrf64 = npt.NDArray[f64]

# Numpy vector types.
type NpVectorNf64 = Annotated[NpArrf64, Literal["N"]]
type NpVector1f64 = Annotated[NpArrf64, Literal["1"]]
type NpVector2f64 = Annotated[NpArrf64, Literal["2"]]
type NpVector3f64 = Annotated[NpArrf64, Literal["3"]]
type NpVector4f64 = Annotated[NpArrf64, Literal["4"]]

# 2D matrix types.
type NpMatrixNMf64 = Annotated[NpArrf64, Literal["N", "M"]]
type NpMatrixN3f64 = Annotated[NpArrf64, Literal["N", "3"]]
type NpMatrixN4f64 = Annotated[NpArrf64, Literal["N", "4"]]

# Time stuff.
type TimesVector = NpVectorNf64  # Time in seconds

# Control.
type PositionsVector = NpVectorNf64
type VelocitiesVector = NpVectorNf64
type StateVector = NpVectorNf64
type GainsVector = NpVectorNf64
type ControlSignalVector = NpVectorNf64

type JointPositionsVector = NpVectorNf64
type JointVelocitiesVector = NpVectorNf64
type JointStateVector = NpVectorNf64

type GripperPositionsVector = NpVectorNf64
type GripperVelocitiesVector = NpVectorNf64
