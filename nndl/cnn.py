import numpy as np

from cs231n.fast_layers import (
    conv_forward_fast,
    conv_backward_fast,
    max_pool_forward_fast,
    max_pool_backward_fast,
)

from nndl.layers import (
    affine_forward,
    affine_backward,
    relu_forward,
    relu_backward,
    softmax_loss,
    batchnorm_forward,
    batchnorm_backward,
)
from nndl.conv_layers import (
    conv_forward_naive,
    conv_backward_naive,
    max_pool_forward_naive,
    max_pool_backward_naive,
    spatial_batchnorm_forward,
    spatial_batchnorm_backward,
)
from nndl.layer_utils import (
    affine_relu_forward,
    affine_relu_backward,
)
from nndl.conv_layer_utils import (
    conv_relu_forward,
    conv_relu_backward,
    conv_relu_pool_forward,
    conv_relu_pool_backward,
)


""" 
This code was originally written for CS 231n at Stanford University
(cs231n.stanford.edu).  It has been modified in various areas for use in the
ECE 239AS class at UCLA.  This includes the descriptions of what code to
implement as well as some slight potential changes in variable names to be
consistent with class nomenclature.  We thank Justin Johnson & Serena Yeung for
permission to use this code.  To see the original version, please visit
cs231n.stanford.edu.  
"""


class ThreeLayerConvNet:
    """
    A three-layer convolutional network with the following architecture:

    conv - relu - 2x2 max pool - affine - relu - affine - softmax

    The network operates on minibatches of data that have shape (N, C, H, W)
    consisting of N images, each with height H and width W and with C input
    channels.
    """

    def __init__(
        self,
        input_dim: tuple[int, int, int] = (3, 32, 32),
        num_filters: int = 32,
        filter_size: int = 7,
        hidden_dim: int = 100,
        num_classes: int = 10,
        weight_scale: float = 1e-3,
        reg: float = 0.0,
        dtype: np.dtype = np.float32,
        use_batchnorm: bool = False,
    ):
        """
        Initialize a new network.

        Inputs:
        - input_dim: Tuple (C, H, W) giving size of input data
        - num_filters: Number of filters to use in the convolutional layer
        - filter_size: Size of filters to use in the convolutional layer
        - hidden_dim: Number of units to use in the fully-connected hidden layer
        - num_classes: Number of scores to produce from the final affine layer.
        - weight_scale: Scalar giving standard deviation for random initialization
          of weights.
        - reg: Scalar giving L2 regularization strength
        - dtype: numpy datatype to use for computation.
        """
        self.use_batchnorm = use_batchnorm
        self.params = {}
        self.reg = reg
        self.dtype = dtype

        # ================================================================ #
        #   Initialize the weights and biases of a three layer CNN. To initialize:
        #     - the biases should be initialized to zeros.
        #     - the weights should be initialized to a matrix with entries
        #         drawn from a Gaussian distribution with zero mean and
        #         standard deviation given by weight_scale.
        # ================================================================ #

        C, H, W = input_dim
        F = num_filters
        
        # 1. Convolutional Layer Weights (W1) and Bias (b1)
        # Shape: (F, C, HH, WW)
        self.params['W1'] = weight_scale * np.random.randn(F, C, filter_size, filter_size)
        self.params['b1'] = np.zeros(F)
        
        # Calculate the size of the flattened output after Conv + Pool
        # Conv layer pad ensures H and W remain the same.
        # Max Pool (2x2, stride 2) halves the H and W.
        H_out = H // 2
        W_out = W // 2
        pool_output_dim = F * H_out * W_out
        
        # 2. Affine Layer Weights (W2) and Bias (b2)
        # Shape: (pool_output_dim, hidden_dim)
        self.params['W2'] = weight_scale * np.random.randn(pool_output_dim, hidden_dim)
        self.params['b2'] = np.zeros(hidden_dim)
        
        # 3. Output Affine Layer Weights (W3) and Bias (b3)
        # Shape: (hidden_dim, num_classes)
        self.params['W3'] = weight_scale * np.random.randn(hidden_dim, num_classes)
        self.params['b3'] = np.zeros(num_classes)
        # ================================================================ #

        for k, v in self.params.items():
            self.params[k] = v.astype(dtype)

    def loss(self, X, y=None):
        """
        Evaluate loss and gradient for the three-layer convolutional network.

        Input / output: Same API as TwoLayerNet in fc_net.py.
        """
        W1, b1 = self.params["W1"], self.params["b1"]
        W2, b2 = self.params["W2"], self.params["b2"]
        W3, b3 = self.params["W3"], self.params["b3"]

        # pass conv_param to the forward pass for the convolutional layer
        filter_size = W1.shape[2]
        conv_param = {"stride": 1, "pad": (filter_size - 1) // 2}

        # pass pool_param to the forward pass for the max-pooling layer
        pool_param = {"pool_height": 2, "pool_width": 2, "stride": 2}

        scores = None

        # ================================================================ #
        #   Implement the forward pass of the three layer CNN.  Store the output
        #   scores as the variable "scores".
        # ================================================================ #

        # Forward Pass
        # Layer 1: Conv -> ReLU -> Pool
        out_pool, cache_pool = conv_relu_pool_forward(X, W1, b1, conv_param, pool_param)
        
        # Layer 2: Affine -> ReLU
        out_affine_relu, cache_affine_relu = affine_relu_forward(out_pool, W2, b2)
        
        # Layer 3: Affine (Output Scores)
        scores, cache_scores = affine_forward(out_affine_relu, W3, b3)
        # ================================================================ #

        if y is None:
            return scores

        loss, grads = 0, {}
        # ================================================================ #
        #   Implement the backward pass of the three layer CNN.  Store the grads
        #   in the grads dictionary, exactly as before (i.e., the gradient of
        #   self.params[k] will be grads[k]).  Store the loss as "loss", and
        #   don't forget to add regularization on ALL weight matrices.
        # ================================================================ #

        # Compute Softmax Loss
        loss, dscores = softmax_loss(scores, y)
        
        # Add L2 Regularization to the loss
        loss += 0.5 * self.reg * (np.sum(W1**2) + np.sum(W2**2) + np.sum(W3**2))
        
        # Backward Pass
        
        # Layer 3: Affine Backward
        dout_affine_relu, dW3, db3 = affine_backward(dscores, cache_scores)
        grads['W3'] = dW3 + self.reg * W3
        grads['b3'] = db3
        
        # Layer 2: Affine -> ReLU Backward
        dout_pool, dW2, db2 = affine_relu_backward(dout_affine_relu, cache_affine_relu)
        grads['W2'] = dW2 + self.reg * W2
        grads['b2'] = db2
        
        # Layer 1: Conv -> ReLU -> Pool Backward
        dx, dW1, db1 = conv_relu_pool_backward(dout_pool, cache_pool)
        grads['W1'] = dW1 + self.reg * W1
        grads['b1'] = db1
        # ================================================================ #

        return loss, grads
