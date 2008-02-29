//  $Id$
//
//  SuperTuxKart - a fun racing game with go-kart
//  Copyright (C) 2004 Steve Baker <sjbaker1@airmail.net>
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the GNU General Public License
//  as published by the Free Software Foundation; either version 2
//  of the License, or (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

#ifndef HEADER_MATERIALMANAGER_H
#define HEADER_MATERIALMANAGER_H

#include <plib/ssg.h>
#include <string>
#include <vector>

class Material;

class MaterialManager
{
private:

    char   *parseFileName(char **str);
    int     parseMaterial(FILE *fd);
    void    parseMaterialFile(const std::string& filename);
    int     m_shared_material_index;

    std::vector<Material*> m_materials;
public:
    MaterialManager();
    void      loadMaterial    ();
    void      reInit          ();
    int       addEntity       (Material *m);
    Material *getMaterial     (ssgLeaf *lf);
//    Material *getMaterial     (const char *texname);
    Material *getMaterial     (const std::string& t, bool is_full_path=false);
    bool      pushTempMaterial(const std::string& filename);
    void      popTempMaterial ();
};

extern ssgState *fuzzy_gst, *herringbones_gst;

ssgState *getAppState ( char *fname ) ;
extern MaterialManager *material_manager;

#endif

/* EOF */
