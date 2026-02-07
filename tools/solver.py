import sys

import numpy as np
from scipy.optimize import linprog


def parse_file(filename):
    with open(filename, 'r') as f:
        lines = [line.strip() for line in f if line.strip()]

    direction = lines[0].strip()
    if direction not in ('min', 'max'):
        raise ValueError("First line must be 'min' or 'max'")
    is_min = (direction == 'min')

    n = int(lines[1])

    c = list(map(float, lines[2].split()))

    num_constraints = int(lines[3])

    A_ub = []
    b_ub = []
    A_eq = []
    b_eq = []

    for i in range(4, 4 + num_constraints):
        parts = lines[i].split()
        if len(parts) < 2:
            raise ValueError(f"Invalid constraint line: {lines[i]}")
        rhs = float(parts[-1])
        sign = parts[-2]
        coeffs = list(map(float, parts[:-2]))

        if len(coeffs) != len(c):
            raise ValueError(f"Constraint {i - 2}: number of coefficients ({len(coeffs)}) != variables ({len(c)})")

        if sign == '<=':
            A_ub.append(coeffs)
            b_ub.append(rhs)
        elif sign == '>=':
            A_ub.append([-x for x in coeffs])
            b_ub.append(-rhs)
        elif sign == '=':
            A_eq.append(coeffs)
            b_eq.append(rhs)
        else:
            raise ValueError(f"Unknown relation '{sign}' in constraint: {lines[i]}")

    var_cons = lines[4 + num_constraints].split()
    if len(var_cons) != len(c):
        raise ValueError(f"Variable constraints count ({len(var_cons)}) != variables ({len(c)})")

    return is_min, np.array(c), A_ub, b_ub, A_eq, b_eq, var_cons


def transform_to_standard(c, A_ub, b_ub, A_eq, b_eq, var_cons):
    """
    Преобразует задачу к стандартной форме для scipy:
      min c_new^T x_new
      s.t. A_ub_new x_new <= b_ub_new
           A_eq_new x_new = b_eq_new
           x_new >= 0

    Обрабатывает:
      - max → min via -c
      - x_i <= 0 → замена y_i = -x_i, y_i >= 0
      - x_i free → x_i = x_i^+ - x_i^-, x_i^+, x_i^- >= 0
    """
    n_orig = len(c)
    new_c = []

    # Индексы новых переменных
    new_var_count = 0
    var_map = []  # для каждой исходной переменной: список (коэф, new_index)

    for i in range(n_orig):
        cons = var_cons[i]
        coeff = c[i]
        if cons == '>=0':
            # x_i >= 0 do nothing
            var_map.append([(1.0, new_var_count)])
            new_c.append(coeff)
            new_var_count += 1
        elif cons == '<=0':
            # x_i <= 0 replace: x_i = -y_i, y_i >= 0
            var_map.append([(-1.0, new_var_count)])
            new_c.append(-coeff)
            new_var_count += 1
        elif cons == 'free':
            # x_i -> x_i^+ - x_i^-
            var_map.append([(1.0, new_var_count), (-1.0, new_var_count + 1)])
            new_c.extend([coeff, -coeff])
            new_var_count += 2
        else:
            raise ValueError(f"Unknown variable constraint: {cons}")

    # Обновляем ограничения
    def expand_constraint(row):
        new_row = [0.0] * new_var_count
        for i, a in enumerate(row):
            for factor, idx in var_map[i]:
                new_row[idx] += a * factor
        return new_row

    new_A_ub = [expand_constraint(row) for row in A_ub]
    new_A_eq = [expand_constraint(row) for row in A_eq]

    return np.array(new_c), new_A_ub, b_ub, new_A_eq, b_eq


def main():
    if len(sys.argv) != 2:
        filename = input("Provide filename: ")
    else:
        filename = sys.argv[1]
    try:
        is_min, c, A_ub, b_ub, A_eq, b_eq, var_cons = parse_file(filename)
    except Exception as e:
        print(f"Error parsing file: {e}")
        sys.exit(1)

    try:
        c_new, A_ub_new, b_ub_new, A_eq_new, b_eq_new = transform_to_standard(
            c, A_ub, b_ub, A_eq, b_eq, var_cons
        )
    except Exception as e:
        print(f"Error transforming to standard form: {e}")
        sys.exit(1)

    # Если максимизация — меняем знак целевой функции
    if not is_min:
        c_new = -c_new

    # Решаем
    bounds = [(0, None)] * len(c_new)  # все новые переменные >= 0

    res = linprog(
        c=c_new,
        A_ub=A_ub_new if A_ub_new else None,
        b_ub=b_ub_new if b_ub_new else None,
        A_eq=A_eq_new if A_eq_new else None,
        b_eq=b_eq_new if b_eq_new else None,
        bounds=bounds,
        method='highs'
    )

    if not res.success:
        print("No feasible solution found.")
        print("Status:", res.message)
        sys.exit(1)

    # Восстанавливаем исходные переменные
    n_orig = len(c)
    x_orig = [0.0] * n_orig
    idx = 0
    for i in range(n_orig):
        cons = var_cons[i]
        if cons == '>=0':
            x_orig[i] = res.x[idx]
            idx += 1
        elif cons == '<=0':
            x_orig[i] = -res.x[idx]
            idx += 1
        elif cons == 'free':
            x_plus = res.x[idx]
            x_minus = res.x[idx + 1]
            x_orig[i] = x_plus - x_minus
            idx += 2

    objective_value = np.dot(c, x_orig)
    if not is_min:
        objective_value = -objective_value

    print("Optimal solution found:")
    print("x =", " ".join(f"{xi:.6g}" for xi in x_orig))
    print("x' =", " ".join(f"{xi:.6g}" for xi in res.x))
    print("Objective value =", f"{objective_value:.6g}")


if __name__ == "__main__":
    main()
