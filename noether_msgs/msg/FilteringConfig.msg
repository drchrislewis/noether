# This message contains parameters for the various mesh filters in VTK
bool enable_filtering

# Parameters for the vtkWindowedSincPolyDataFilter
int32 windowed_sinc_iterations
float64 windowed_sinc_pass_band
float64 windowed_sinc_feature_angle
float64 windowed_sinc_edge_angle
bool windowed_sinc_edge_smoothing
bool windowed_sinc_boundary_smoothing
bool windowed_sinc_nonmanifold_smoothing
bool windowed_sinc_normalize_coordinates

# Parameters for the vtkSmoothPolyDataFilter (Laplacian smoothing)
int32 laplacian_iterations
float64 laplacian_relaxation_factor
float64 laplacian_edge_angle
bool laplacian_edge_smoothing
bool laplacian_boundary_smoothing
