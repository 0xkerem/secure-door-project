import numpy as np

def cosine_similarity(a, b):
    """
    Compute the cosine similarity between two vectors or matrices.
    a: np.ndarray, shape (n_samples_a, n_features) or (n_features,)
    b: np.ndarray, shape (n_samples_b, n_features) or (n_features,)
    Returns:
        np.ndarray: Cosine similarity matrix of shape (n_samples_a, n_samples_b)
    """
    a = np.atleast_2d(a)
    b = np.atleast_2d(b)
    a_norm = np.linalg.norm(a, axis=1, keepdims=True)
    b_norm = np.linalg.norm(b, axis=1, keepdims=True)
    a_normalized = a / (a_norm + 1e-10)
    b_normalized = b / (b_norm + 1e-10)
    return np.dot(a_normalized, b_normalized.T)

def normalize(X):
    """
    Normalize a vector or matrix to unit norm.
    X: np.ndarray, shape (n_samples, n_features) or (n_features,)
    Returns:
        np.ndarray: Normalized array with unit norm along axis 1 (rows).
    """
    X = np.atleast_2d(X)
    norms = np.linalg.norm(X, axis=1, keepdims=True)
    return X / (norms + 1e-10)