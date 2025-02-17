/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef VOXEL_MAP_HPP
#define VOXEL_MAP_HPP

#include "voxel_dilater.hpp"
#include <memory>
#include <vector>
#include <Eigen/Eigen>

namespace voxel_map
{

    constexpr uint8_t Unoccupied = 0;
    constexpr uint8_t Occupied = 1;
    constexpr uint8_t Dilated = 2;

    class VoxelMap
    {

    public:
        VoxelMap() = default;
        VoxelMap(const Eigen::Vector3i &size,
                 const Eigen::Vector3d &origin,
                 const double &voxScale)
            : mapSize(size),
              o(origin),
              scale(voxScale),
              voxNum(mapSize.prod()),
              step(1, mapSize(0), mapSize(1) * mapSize(0)),
              oc(o + Eigen::Vector3d::Constant(0.5 * scale)),
              bounds((mapSize.array() - 1) * step.array()),
              stepScale(step.cast<double>().cwiseInverse() * scale),
              voxels(voxNum, Unoccupied) {}

        typedef std::shared_ptr<VoxelMap> Ptr;

    private:
        Eigen::Vector3i mapSize;
        Eigen::Vector3d o;
        double scale;
        int voxNum;
        Eigen::Vector3i step;
        Eigen::Vector3d oc;
        Eigen::Vector3i bounds;
        Eigen::Vector3d stepScale;
        std::vector<uint8_t> voxels;
        std::vector<Eigen::Vector3i> surf;

    public:
        inline Eigen::Vector3i getSize(void) const
        {
            return mapSize;
        }

        inline double getScale(void) const
        {
            return scale;
        }

        inline Eigen::Vector3d getOrigin(void) const
        {
            return o;
        }

        inline Eigen::Vector3d getCorner(void) const
        {
            return mapSize.cast<double>() * scale + o;
        }

        inline const std::vector<uint8_t> &getVoxels(void) const
        {
            return voxels;
        }

        inline void setOccupied(const Eigen::Vector3d &pos)
        {
            const Eigen::Vector3i id = ((pos - o) / scale).cast<int>();
            if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
                id(0) < mapSize(0) && id(1) < mapSize(1) && id(2) < mapSize(2))
            {
                voxels[id.dot(step)] = Occupied;
            }
        }

        inline void setOccupied(const Eigen::Vector3i &id)
        {
            if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
                id(0) < mapSize(0) && id(1) < mapSize(1) && id(2) < mapSize(2))
            {
                voxels[id.dot(step)] = Occupied;
            }
        }

        inline void dilate(const int &r, Eigen::Vector3i lb, Eigen::Vector3i hb)
        {
            if (r <= 0)
            {
                return;
            }
            else
            {
                boundIndex(lb);
                boundIndex(hb);

                Eigen::Vector3i lb_s = lb.array() * step.array();
                Eigen::Vector3i hb_s = hb.array() * step.array();

                std::vector<Eigen::Vector3i> lvec, cvec;
                lvec.reserve(voxNum);
                cvec.reserve(voxNum);
                int i, j, k, idx;
                bool check;
                for (int x = lb_s(0); x <= hb_s(0); x++)
                {
                    for (int y = lb_s(1); y <= hb_s(1); y += step(1))
                    {
                        for (int z = lb_s(2); z <= hb_s(2); z += step(2))
                        {
                            if (voxels[x + y + z] == Occupied)
                            {
                                VOXEL_DILATER(i, j, k,
                                              x, y, z,
                                              step(1), step(2),
                                              bounds(0), bounds(1), bounds(2),
                                              check, voxels, idx, Dilated, cvec)
                            }
                        }
                    }
                }

                for (int loop = 1; loop < r; loop++)
                {
                    std::swap(cvec, lvec);
                    for (const Eigen::Vector3i &id : lvec)
                    {
                        VOXEL_DILATER(i, j, k,
                                      id(0), id(1), id(2),
                                      step(1), step(2),
                                      bounds(0), bounds(1), bounds(2),
                                      check, voxels, idx, Dilated, cvec)
                    }
                    lvec.clear();
                }

                surf = cvec;
            }
        }

        inline void dilate(const int &r)
        {
            dilate(r, Eigen::Vector3i::Zero(), Eigen::Vector3i(mapSize.array() - 1));
        }

        inline void dilate(const int &r, Eigen::Vector3d lb, Eigen::Vector3d hb)
        {
            dilate(r, posD2I(lb), posD2I(hb));
        }

        inline void getSurfInBox(const Eigen::Vector3i &center,
                                 const int &halfWidth,
                                 std::vector<Eigen::Vector3d> &points) const
        {
            for (const Eigen::Vector3i &id : surf)
            {
                if (std::abs(id(0) - center(0)) <= halfWidth &&
                    std::abs(id(1) / step(1) - center(1)) <= halfWidth &&
                    std::abs(id(2) / step(2) - center(2)) <= halfWidth)
                {
                    points.push_back(id.cast<double>().cwiseProduct(stepScale) + oc);
                }
            }

            return;
        }

        inline void getSurf(std::vector<Eigen::Vector3d> &points) const
        {
            points.reserve(surf.size());
            for (const Eigen::Vector3i &id : surf)
            {
                points.push_back(id.cast<double>().cwiseProduct(stepScale) + oc);
            }
            return;
        }

        inline bool query(const Eigen::Vector3d &pos) const
        {
            const Eigen::Vector3i id = ((pos - o) / scale).cast<int>();
            if (id(2) < 0 || id(2) >= mapSize(2))
            {
                return true;
            }
            // if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
            //     id(0) < mapSize(0) && id(1) < mapSize(1) && id(2) < mapSize(2))
            else if (id(0) >= 0 && id(1) >= 0 && id(0) < mapSize(0) && id(1) < mapSize(1))
            {
                return voxels[id.dot(step)];
            }
            else
            {
                return false;
            }
        }

        inline bool query(const Eigen::Vector3i &id) const
        {
            if (id(2) < 0 || id(2) >= mapSize(2))
            {
                return true;
            }
            // if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
            //     id(0) < mapSize(0) && id(1) < mapSize(1) && id(2) < mapSize(2))
            else if (id(0) >= 0 && id(1) >= 0 && id(0) < mapSize(0) && id(1) < mapSize(1))
            {
                return voxels[id.dot(step)];
            }
            else
            {
                return false;
            }
        }

        inline Eigen::Vector3d posI2D(const Eigen::Vector3i &id) const
        {
            return id.cast<double>() * scale + oc;
        }

        inline Eigen::Vector3i posD2I(const Eigen::Vector3d &pos) const
        {
            return ((pos - o) / scale).cast<int>();
        }

        inline void boundIndex(Eigen::Vector3i &id)
        {
            id(0) = id(0) > 0 ? id(0) : 0; id(0) = id(0) < mapSize(0) - 1 ? id(0) : mapSize(0) - 1;
            id(1) = id(1) > 0 ? id(1) : 0; id(1) = id(1) < mapSize(1) - 1 ? id(1) : mapSize(1) - 1;
            id(2) = id(2) > 0 ? id(2) : 0; id(2) = id(2) < mapSize(2) - 1 ? id(2) : mapSize(2) - 1;
        }

        inline void resetVoxel(Eigen::Vector3i lb, Eigen::Vector3i hb)
        {
            boundIndex(lb);
            boundIndex(hb);

            for (int x = lb(0); x <= hb(0); x++)
            {
                for (int y = lb(0); y <= hb(1); y ++)
                {
                    for (int z = lb(0); z <= hb(2); z ++)
                    {
                        Eigen::Vector3i id(x, y, z);
                        voxels[id.dot(step)] = Unoccupied;
                    }
                }
            }
            surf.clear();
        }

        inline void resetVoxel()
        {
            std::fill(voxels.begin(), voxels.end(), 0);
        }

        inline void resetVoxel(Eigen::Vector3d lb, Eigen::Vector3d hb)
        {
            resetVoxel(posD2I(lb), posD2I(hb));
        }

        inline void changeOrigin(Eigen::Vector3i move_offset)
        {
            int offset = move_offset.dot(step);
            int max_addr = mapSize(0) * mapSize(1) * mapSize(2);

            if (abs(move_offset(0)) >= mapSize(0) || abs(move_offset(1)) >= mapSize(1) || abs(move_offset(2)) >= mapSize(2))
            {
                for (int addr = 0; addr < max_addr; addr++) {
                    voxels[addr] = 0;
                }
            }
            else if (offset > 0)
            {
                for (int addr = 0; addr < max_addr - offset; addr++)
                {
                    voxels[addr] = voxels[addr + offset];
                }

                if (move_offset.z() != 0)
                {
                    for (int addr = max_addr - offset; addr < max_addr; addr++)
                    {
                        voxels[addr] = 0;
                    }
                }
                else if (move_offset.y() != 0)
                {
                    for (int i = 0; i < mapSize(2); i++)
                    {
                        int part_max_addr = (i + 1) * mapSize(1) * mapSize(0);
                        for (int addr = part_max_addr - offset; addr < part_max_addr; addr++)
                        {
                            voxels[addr] = 0;
                        }
                    }
                }
                else if (move_offset.x() != 0)
                {
                    for (int i = 0; i < mapSize(2); i++)
                    {
                        for (int j = 0; j < mapSize(1); j++)
                        {
                            int part_max_addr = i * mapSize(1) * mapSize(0) + (j + 1) * mapSize(0);
                            for (int addr = part_max_addr - offset; addr < part_max_addr; addr++)
                            {
                                voxels[addr] = 0;
                            }
                        }
                    }
                }
            }
            else if (offset < 0)
            {
                for (int addr = max_addr - 1; addr >= -offset; addr--)
                {
                    voxels[addr] = voxels[addr + offset];
                }

                if (move_offset.z() != 0)
                {
                    for (int addr = 0; addr < -offset; addr++)
                    {
                        voxels[addr] = 0;
                    }
                }
                else if (move_offset.y() != 0)
                {
                    for (int i = 0; i < mapSize(2); i++)
                    {
                        int part_max_addr = i * mapSize(1) * mapSize(0);
                        for (int addr = part_max_addr; addr < part_max_addr - offset; addr++)
                        {
                            voxels[addr] = 0;
                        }
                    }
                }
                else if (move_offset.x() != 0)
                {
                    for (int i = 0; i < mapSize(2); i++)
                    {
                        for (int j = 0; j < mapSize(1); j++)
                        {
                            int part_max_addr = i * mapSize(1) * mapSize(0) + j * mapSize(0);
                            for (int addr = part_max_addr; addr < part_max_addr - offset; addr++)
                            {
                                voxels[addr] = 0;
                            }
                        }
                    }
                }
            }
            else
                return;

            o += move_offset.cast<double>() * scale;
            oc = o + Eigen::Vector3d::Constant(0.5 * scale);
        }

        inline void changeOrigin(Eigen::Vector3d move_offset)
        {
            Eigen::Vector3i move_offset_i;
            move_offset_i.x() = move_offset.x() / scale;
            move_offset_i.y() = move_offset.y() / scale;
            move_offset_i.z() = move_offset.z() / scale;
            changeOrigin(move_offset_i);
        }
    };
}

#endif
