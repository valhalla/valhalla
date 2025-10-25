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
  route(query: Query): Response;
  route(query: string): string;

  /**
   * Locate coordinates on the road network
   */
  locate(query: Query): Response;
  locate(query: string): string;

  /**
   * Calculate time-distance matrix
   */
  matrix(query: Query): Response;
  matrix(query: string): string;

  /**
   * Calculate optimized route (traveling salesman problem)
   */
  optimizedRoute(query: Query): Response;
  optimizedRoute(query: string): string;

  /**
   * Calculate isochrones (time/distance contours)
   */
  isochrone(query: Query): Response;
  isochrone(query: string): string;

  /**
   * Match GPS trace to road network
   */
  traceRoute(query: Query): Response;
  traceRoute(query: string): string;

  /**
   * Get detailed attributes for a GPS trace
   */
  traceAttributes(query: Query): Response;
  traceAttributes(query: string): string;

  /**
   * Get elevation data for coordinates
   */
  height(query: Query): Response;
  height(query: string): string;

  /**
   * Check transit availability at locations
   */
  transitAvailable(query: Query): Response;
  transitAvailable(query: string): string;

  /**
   * Get route expansion visualization data
   */
  expansion(query: Query): Response;
  expansion(query: string): string;

  /**
   * Calculate centroid of locations
   */
  centroid(query: Query): Response;
  centroid(query: string): string;

  /**
   * Get Valhalla service status
   */
  status(query: Query): Response;
  status(query: string): string;
}

/**
 * Options for configuring Valhalla
 */
export interface GetConfigOptions {
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
}

/**
 * Returns a default Valhalla configuration with optional customizations
 * @param options - Configuration options
 * @returns Valhalla configuration object
 */
export function getConfig(options?: GetConfigOptions): ValhallaConfig;

/**
 * Valhalla version string
 */
export const VALHALLA_VERSION: string;
