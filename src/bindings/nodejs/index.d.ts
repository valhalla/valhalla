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
 * Valhalla version string
 */
export const VALHALLA_VERSION: string;
