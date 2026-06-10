# MIP 2026

Проект по дисциплине **«Моделирование информационных процессов»**.

## Установка

```bash
git clone https://github.com/nikitanasibullin/mip2025.git
cd mip2025
pip install -r requirements.txt
```

## Запуск

### Двухзвенный манипулятор

```bash
python two-link/pendulum.py
```

## Файлы проекта

### `pendulum.py`
Сценарий для визуализации движения манипулятора к фиксированной целевой точке.

### `two-link/pendulum.py`
Сценарий для управления двухзвенным манипулятором через ползунки `target_x` и `target_z`.

### `two-link/two-link.urdf.xml`
URDF-модель двухзвенного манипулятора.

