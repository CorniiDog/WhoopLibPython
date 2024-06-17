import torch

print("Torch version:", torch.__version__)

# Check if CUDA is available
if torch.cuda.is_available():
    device = torch.device("cuda")  # Use CUDA if available
    print("CUDA is available. Using GPU for computations.")
else:
    device = torch.device("cpu")  # Use CPU if CUDA is not available
    print("CUDA is not available. Using CPU for computations.")
