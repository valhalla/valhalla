// Type definitions for Valhalla Node.js bindings

/**
 * Valhalla configuration object
 */
export type ValhallaConfig = Record<string, any>;

/**
 * Generic query object for Valhalla API methods
 */
export type Query = Record<string, any>;

/**
 * Generic response object from Valhalla API methods
 */
export type Response = Record<string, any>;

/**
 * Valhalla Actor class for making routing requests
 */
export class Actor {
  /**
   * Create a new Valhalla Actor instance
   * @param config - Valhalla configuration object or JSON string
   */
  constructor(config: ValhallaConfig | string);

  /**
   * Create an Actor instance from a config file
   * @param configFile - Path to the config file
   * @returns Promise that resolves to an Actor instance
   */
  static fromConfigFile(configFile: string): Promise<Actor>;

  /**
   * Calculate a route between locations
   */
  route(query: Query): Promise<Response>;
  route(query: string): Promise<string>;

  /**
   * Locate coordinates on the road network
   */
  locate(query: Query): Promise<Response>;
  locate(query: string): Promise<string>;

  /**
   * Calculate time-distance matrix
   */
  matrix(query: Query): Promise<Response>;
  matrix(query: string): Promise<string>;

  /**
   * Calculate optimized route (traveling salesman problem)
   */
  optimizedRoute(query: Query): Promise<Response>;
  optimizedRoute(query: string): Promise<string>;

  /**
   * Calculate isochrones (time/distance contours)
   */
  isochrone(query: Query): Promise<Response>;
  isochrone(query: string): Promise<string>;

  /**
   * Match GPS trace to road network
   */
  traceRoute(query: Query): Promise<Response>;
  traceRoute(query: string): Promise<string>;

  /**
   * Get detailed attributes for a GPS trace
   */
  traceAttributes(query: Query): Promise<Response>;
  traceAttributes(query: string): Promise<string>;

  /**
   * Get elevation data for coordinates
   */
  height(query: Query): Promise<Response>;
  height(query: string): Promise<string>;

  /**
   * Check transit availability at locations
   */
  transitAvailable(query: Query): Promise<Response>;
  transitAvailable(query: string): Promise<string>;

  /**
   * Get route expansion visualization data
   */
  expansion(query: Query): Promise<Response>;
  expansion(query: string): Promise<string>;

  /**
   * Calculate centroid of locations
   */
  centroid(query: Query): Promise<Response>;
  centroid(query: string): Promise<string>;

  /**
   * Get Valhalla service status
   */
  status(query: Query): Promise<Response>;
  status(query: string): Promise<string>;
}

/**
 * Options for configuring Valhalla
 */
export interface ConfigOptions {
  /**
   * The file path (with .tar extension) of the tile extract (mjolnir.tile_extract)
   * @default 'valhalla_tiles.tar'
   */
  tileExtract?: string;
  
  /**
   * The directory path where the graph tiles are stored (mjolnir.tile_dir)
   * @default 'valhalla_tiles'
   */
  tileDir?: string;
  
  /**
   * Whether you want to see Valhalla's logs on stdout (mjolnir.logging)
   * @default false
   */
  verbose?: boolean;

  /**
   * Additional arguments to pass to valhalla_build_config script
   * @default []
   */
  additionalArgs?: string[];
}

/**
 * Generates Valhalla configuration by executing valhalla_build_config script
 * @param options - Configuration options
 * @returns Promise that resolves to Valhalla configuration object
 */
export function getConfig(options?: ConfigOptions): Promise<ValhallaConfig>;

/**
 * Valhalla version string
 */
export const VALHALLA_VERSION: string;
