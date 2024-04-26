function a_hat = normalizeVector(a_vect)
    a = norm(a_vect);
    a_hat = a_vect/a;
end