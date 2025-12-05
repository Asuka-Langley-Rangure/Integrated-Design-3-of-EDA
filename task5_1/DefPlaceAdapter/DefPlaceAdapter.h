#ifndef DEFPLACEADAPTER_H
#define DEFPLACEADAPTER_H

#include "placedata.h"
#include "db.h"   // 就是你刚才发的 PlacementDB 定义

namespace DefPlaceAdapter {

    /// PlacementDB -> PlaceData
    void buildPlaceDataFromPlacementDB(const PlacementDB& src, PlaceData& dst);

    /// PlaceData -> PlacementDB
    void buildPlacementDBFromPlaceData(const PlaceData& src, PlacementDB& dst);
}

#endif // DEFPLACEADAPTER_H
