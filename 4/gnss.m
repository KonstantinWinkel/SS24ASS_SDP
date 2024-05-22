X = load('glonass.txt')

[n_sats, nc] = size(X)

pos = zeros(1,3)

c = 299792458

for Z = 1:20
    distance_vectors= zeros(n_sats, 3)
    rho_k = zeros(n_sats,1)
    b = zeros(n_sats,1)
    A = zeros(n_sats,4)
    # compute distances between sats and receiver
    for i = 1:n_sats
      distance_vectors(i,1) = X(i, 2) - pos(1)
      distance_vectors(i,2) = X(i, 3) - pos(2)
      distance_vectors(i,3) = X(i, 4) - pos(3)
    end

    # equation 2
    for i = 1:n_sats
      rho_k(i,1) = sqrt(distance_vectors(i,1)^2 + distance_vectors(i,2)^2 + distance_vectors(i,3)^2)
    end

    #A and B
    for i = 1:n_sats
        A(i,1) = -(X(i,2)-pos(1)) / rho_k(i)
        A(i,2) = -(X(i,3)-pos(2)) / rho_k(i)
        A(i,3) = -(X(i,4)-pos(3)) / rho_k(i)
        A(i,4) = 1

        b(i) = X(i,1) - rho_k(i) + c * X(i,5)
    end

    #equation 9
    d_pos = inv(A' * A) * A' * b

    pos(1) += d_pos(1)
    pos(2) += d_pos(2)
    pos(3) += d_pos(3)

end
