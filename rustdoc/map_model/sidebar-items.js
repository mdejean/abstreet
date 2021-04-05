initSidebarItems({"constant":[["MAX_BIKE_SPEED",""],["MAX_WALKING_SPEED",""],["NORMAL_LANE_THICKNESS",""],["PARKING_LOT_SPOT_LENGTH","From some manually audited cases in Seattle, the length of parallel street parking spots is a bit different than the length in parking lots, so set a different value here."],["SERVICE_ROAD_LANE_THICKNESS",""],["SHOULDER_THICKNESS",""],["SIDEWALK_THICKNESS",""]],"enum":[["AmenityType","Businesses are categorized into one of these types."],["AreaType",""],["BuildingType",""],["Direction",""],["DrivingSide",""],["EditCmd",""],["EditIntersection",""],["IntersectionType",""],["LaneType",""],["OffstreetParking","Represent no parking as Private(0, false)."],["PathConstraints","Who's asking for a path?"],["PathStep",""],["StageType",""],["Traversable","Either a lane or a turn, where most movement happens."],["TurnPriority",""],["TurnType",""]],"mod":[["city",""],["connectivity",""],["edits","Once a Map exists, the player can edit it in the UI (producing `MapEdits` in-memory), then save the changes to a file (as `PermanentMapEdits`). See https://a-b-street.github.io/docs/map/edits.html."],["make","See https://a-b-street.github.io/docs/map/importing/index.html for an overview. This module covers the RawMap->Map stage."],["map","A bunch of (mostly read-only) queries on a Map."],["objects",""],["osm","Useful utilities for working with OpenStreetMap."],["pathfind","Everything related to pathfinding through a map for different types of agents."],["raw","The convert_osm crate produces a RawMap from OSM and other data. Storing this intermediate structure is useful to iterate quickly on parts of the map importing pipeline without having to constantly read .osm files, and to visualize the intermediate state with map_editor."],["traversable",""]],"struct":[["AccessRestrictions",""],["Amenity","A business located inside a building."],["Area","Areas are just used for drawing."],["AreaID",""],["Building","A building has connections to the road and sidewalk, may contain commercial amenities, and have off-street parking."],["BuildingID",""],["BusRoute",""],["BusRouteID",""],["BusStop",""],["BusStopID",""],["City","A single city (like Seattle) can be broken down into multiple boundary polygons (udistrict, ballard, downtown, etc). The load map screen uses this struct to display the entire city."],["CompressedMovementID","This is cheaper to store than a MovementID. It simply indexes into the list of movements."],["ControlStopSign",""],["ControlTrafficSignal","A traffic signal consists of a sequence of Stages that repeat in a cycle. Most Stages last for a fixed duration. During a single Stage, some movements are protected (can proceed with the highest priority), while others are permitted (have to yield before proceeding)."],["DirectedRoadID",""],["EditEffects",""],["EditRoad",""],["Intersection","An intersection connects roads. Most have >2 roads and are controlled by stop signs or traffic signals. Roads that lead to the boundary of the map end at border intersections, with only that one road attached."],["IntersectionCluster","This only applies to VehiclePathfinder; walking through these intersections is nothing special."],["IntersectionID",""],["Lane","A road segment is broken down into individual lanes, which have a LaneType."],["LaneID",""],["Map",""],["MapConfig",""],["MapEdits","Represents changes to a map. Note this isn't serializable -- that's what `PermanentMapEdits` does."],["Movement","A Movement groups all turns from one road to another, letting traffic signals operate at a higher level of abstraction. This is used for pathfinding and traffic signals currently; other places focus instead on turns."],["MovementID","A movement is like a turn, but with less detail -- it identifies a movement from one directed road to another. One road usually has 4 crosswalks, each a singleton Movement. We need all of the information here to keep each crosswalk separate."],["NamePerLanguage","None corresponds to the native name"],["ParkingLot","Parking lots have some fixed capacity for cars, and are connected to a sidewalk and road."],["ParkingLotID",""],["Path",""],["PathRequest",""],["PermanentMapEdits","MapEdits are converted to this before serializing. Referencing things like LaneID in a Map won't work if the basemap is rebuilt from new OSM data, so instead we use stabler OSM IDs that're less likely to change."],["Position","Represents a specific point some distance along a lane."],["Road","A Road represents a segment between exactly two Intersections. It contains Lanes as children."],["RoadID",""],["RoadWithStopSign",""],["RoutingParams","Tuneable parameters for all types of routing."],["Stage",""],["Turn","A Turn leads from the end of one Lane to the start of another. (Except for pedestrians; sidewalks are bidirectional.)"],["TurnID","Turns are uniquely identified by their (src, dst) lanes and their parent intersection. Intersection is needed to distinguish crosswalks that exist at two ends of a sidewalk."],["UberTurn",""],["Zone","A contiguous set of roads with access restrictions. This is derived from all the map's roads and kept cached for performance."]]});