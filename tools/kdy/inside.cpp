// Быстрая проверка «точка внутри детали» для посадки серво (tools/kdy/fit_kdy.py вызывает этот модуль).
// Сборка: g++ -O2 -o tools/kdy/inside tools/kdy/inside.cpp
// stdin: путь к бинарному STL, матрица 4x4 (деталь -> система серво, по строкам), N, затем N точек x y z.
// stdout: N символов '0'/'1' — точка внутри замкнутого меша (чётность пересечений луча +Z).
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <iostream>

struct Tri { float a[3], b[3], c[3]; float lo[2], hi[2]; };

int main() {
    std::ios::sync_with_stdio(false);
    std::string path; std::cin >> path;
    double M[16]; for (double &m : M) std::cin >> m;
    FILE *f = fopen(path.c_str(), "rb"); if (!f) { fprintf(stderr, "нет файла %s\n", path.c_str()); return 1; }
    char hdr[80]; uint32_t n = 0;
    if (fread(hdr, 1, 80, f) != 80 || fread(&n, 4, 1, f) != 1) return 1;
    std::vector<Tri> T(n);
    for (uint32_t i = 0; i < n; i++) {
        float buf[12]; uint16_t attr;
        if (fread(buf, 4, 12, f) != 12 || fread(&attr, 2, 1, f) != 1) return 1;
        float *v[3] = {T[i].a, T[i].b, T[i].c};
        for (int k = 0; k < 3; k++) {
            const float *p = buf + 3 + 3 * k;
            for (int r = 0; r < 3; r++) v[k][r] = float(M[4 * r] * p[0] + M[4 * r + 1] * p[1] + M[4 * r + 2] * p[2] + M[4 * r + 3]);
        }
        for (int r = 0; r < 2; r++) { T[i].lo[r] = std::min({T[i].a[r], T[i].b[r], T[i].c[r]}); T[i].hi[r] = std::max({T[i].a[r], T[i].b[r], T[i].c[r]}); }
    }
    fclose(f);
    // сетка по XY: ячейка 2 мм, в ячейке — индексы треугольников, чьи AABB её задевают
    float gx0 = 1e9, gy0 = 1e9, gx1 = -1e9, gy1 = -1e9;
    for (auto &t : T) { gx0 = std::min(gx0, t.lo[0]); gy0 = std::min(gy0, t.lo[1]); gx1 = std::max(gx1, t.hi[0]); gy1 = std::max(gy1, t.hi[1]); }
    const float cs = 2.0f; int nx = int((gx1 - gx0) / cs) + 1, ny = int((gy1 - gy0) / cs) + 1;
    std::vector<std::vector<uint32_t>> grid(size_t(nx) * ny);
    for (uint32_t i = 0; i < n; i++) {
        int i0 = int((T[i].lo[0] - gx0) / cs), i1 = int((T[i].hi[0] - gx0) / cs), j0 = int((T[i].lo[1] - gy0) / cs), j1 = int((T[i].hi[1] - gy0) / cs);
        for (int a = i0; a <= i1; a++) for (int b = j0; b <= j1; b++) grid[size_t(b) * nx + a].push_back(i);
    }
    size_t N; std::cin >> N; std::string out(N, '0');
    for (size_t q = 0; q < N; q++) {
        double p[3]; std::cin >> p[0] >> p[1] >> p[2];
        int a = int((p[0] - gx0) / cs), b = int((p[1] - gy0) / cs);
        if (a < 0 || b < 0 || a >= nx || b >= ny) continue;
        int c = 0;
        for (uint32_t i : grid[size_t(b) * nx + a]) {
            const Tri &t = T[i];
            if (p[0] < t.lo[0] || p[0] > t.hi[0] || p[1] < t.lo[1] || p[1] > t.hi[1]) continue;
            double d = (t.b[1] - t.c[1]) * (t.a[0] - t.c[0]) + (t.c[0] - t.b[0]) * (t.a[1] - t.c[1]); if (std::fabs(d) < 1e-12) continue;
            double l1 = ((t.b[1] - t.c[1]) * (p[0] - t.c[0]) + (t.c[0] - t.b[0]) * (p[1] - t.c[1])) / d;
            double l2 = ((t.c[1] - t.a[1]) * (p[0] - t.c[0]) + (t.a[0] - t.c[0]) * (p[1] - t.c[1])) / d, l3 = 1 - l1 - l2;
            if (l1 < 0 || l2 < 0 || l3 < 0) continue;
            if (l1 * t.a[2] + l2 * t.b[2] + l3 * t.c[2] > p[2]) c++;
        }
        if (c & 1) out[q] = '1';
    }
    std::cout << out << '\n';
    return 0;
}
