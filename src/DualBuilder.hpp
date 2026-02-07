#pragma once
#ifndef DUAL_BUILDER_H
#define DUAL_BUILDER_H

#include "LinearProgram.hpp"

/**
 * DualBuilder - строит двойственную задачу ТОЛЬКО для задач в общей форме (формула 4.1).
 *
 * ВАЖНО: Функция ТРЕБУЕТ, чтобы входная задача была в общей форме!
 * Перед вызовом убедитесь, что задача преобразована через FormConverter::to_general_form().
 *
 * Правила двойственности (для primal задачи на МИНИМИЗАЦИЮ в общей форме):
 *
 *   Тип ограничения primal  →  Ограничение на переменную dual
 *     aᵀx ≥ b               →  y ≥ 0
 *     aᵀx = b               →  y free
 *
 *   Ограничение на переменную primal  →  Тип ограничения dual
 *     x ≥ 0                            →  aᵀy ≥ c
 *     x free                           →  aᵀy = c
 *
 * Для задач на максимизацию все неравенства меняют направление, но в общей форме
 * всегда используется минимизация, поэтому правила фиксированы.
 */
class DualBuilder {
public:
    // ЕДИНСТВЕННАЯ функция для построения двойственной задачи
    // Требование: primal ДОЛЖНА быть в общей форме (формула 4.1)
    static LinearProgram build_dual(const LinearProgram& primal_general_form);
};

#endif // DUAL_BUILDER_H