import numpy as np
import time
from pathlib import Path


def FarthestPointSampling_ForBatch(xyz, npoint):
    B, N, C = xyz.shape
 
    centroids = np.zeros((B, npoint))
    distance = np.ones((B, N)) * 1e10
 
    batch_indices = np.arange(B)
 
    barycenter = np.sum((xyz), 1)
    barycenter = barycenter/xyz.shape[1]
    barycenter = barycenter.reshape(B, 1, 3)
 
    dist = np.sum((xyz - barycenter) ** 2, -1)
    farthest = np.argmax(dist,1)
 
    for i in range(npoint):
        # print("-------------------------------------------------------")
        # print("The %d farthest pts %s " % (i, farthest))
        centroids[:, i] = farthest
        centroid = xyz[batch_indices, farthest, :].reshape(B, 1, 3)
        dist = np.sum((xyz - centroid) ** 2, -1)
        mask = dist < distance
        distance[mask] = dist[mask]
        farthest = np.argmax(distance, -1)
    return centroids.astype('int')


if __name__ == "__main__":
    from av2.map.map_api import ArgoverseStaticMap, RasterLayerType, DrivableAreaMapLayer
    from av2.structures.cuboid import CuboidList
    from av2.structures.sweep import Sweep
    from av2.utils.io import TimestampedCitySE3EgoPoses, read_city_SE3_ego, read_feather, read_img
    from pyarrow import feather
    
    dataset_root = Path('/heyufei1/huyinghao/dataset/samples/av2/val')
    log_id = '02678d04-cc9f-3148-9f95-1ba66347dff9'
    timestamp_ns = 315969904359876000
    
    log_dir = dataset_root / log_id
    sensor_dir = log_dir / "sensors"
    lidar_feather_path = sensor_dir / "lidar" / f"{timestamp_ns}.feather"
    sweep = Sweep.from_feather(lidar_feather_path=lidar_feather_path)
    points = np.concatenate([sweep.xyz, sweep.intensity.reshape((-1, 1))], axis=1)
    
    timestamp_city_SE3_ego_dict = read_city_SE3_ego(log_dir=log_dir)
    city_SE3_ego = timestamp_city_SE3_ego_dict[timestamp_ns]
    
    
    start_t = time.time()
    map_dir = log_dir / 'map'
    static_map = ArgoverseStaticMap.from_map_dir(map_dir, build_raster=False)
    
    drivable_areas = list(static_map.vector_drivable_areas.values())
    static_map.raster_drivable_area_layer = DrivableAreaMapLayer.from_vector_data(drivable_areas=drivable_areas)
    
    lidar_points_city = city_SE3_ego.transform_point_cloud(points[:, :3])
    is_da_boolean_arr = static_map.get_raster_layer_points_boolean(lidar_points_city, layer_name=RasterLayerType.DRIVABLE_AREA)
    points = points[is_da_boolean_arr]
    end_t = time.time()
    print(f"Load map info and Raster map, Time Cost:{end_t - start_t}s")
    
    
    batch_data = []
    batch_data.append(points[:, :3])
    batch_data = np.array(batch_data)
    
    npoint = int(points.shape[0] / 2)
    start_t = time.time()
    centroids_index = FarthestPointSampling_ForBatch(batch_data, npoint)
    end_t = time.time()
    
    print(f"Points Num: {points.shape[0]}, N: {npoint}, Time Cost:{end_t - start_t}s")
    pass