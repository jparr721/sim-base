import numpy as np

def tridiagonal_matrix():
    a = np.random.rand(5) * 2
    print("a", a)
    b = np.random.rand(5)
    print("b", b)
    c = np.random.rand(5) * 2
    print("c", c)


    A = np.zeros((5, 5))
    ii, jj = np.indices(A.shape)

    A[ii == jj] = a
    A[ii == jj-1] = b[:4]
    A[ii == jj+1] = c[:4]


    return A

if __name__ == "__main__":
    b = np.random.randint(0, 10, 5)
    A = tridiagonal_matrix()

    print("b", b)
    print("A", A)

    x = np.linalg.solve(A, b)
    print("x: ", x)
