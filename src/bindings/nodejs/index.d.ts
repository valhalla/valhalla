export type ValhallaConfig = Record<string, any>;
export type Query = Record<string, any>;
export type Response = Record<string, any>;

export class Actor {
  /**
   * @param config - Valhalla configuration object or JSON string
   */
  constructor(config: ValhallaConfig | string);

  static fromConfigFile(configFile: string): Promise<Actor>;

  route(query: Query): Promise<Response>;
  route(query: string): Promise<string>;

  locate(query: Query): Promise<Response>;
  locate(query: string): Promise<string>;

  matrix(query: Query): Promise<Response>;
  matrix(query: string): Promise<string>;

  optimizedRoute(query: Query): Promise<Response>;
  optimizedRoute(query: string): Promise<string>;

  isochrone(query: Query): Promise<Response>;
  isochrone(query: string): Promise<string>;

  traceRoute(query: Query): Promise<Response>;
  traceRoute(query: string): Promise<string>;

  traceAttributes(query: Query): Promise<Response>;
  traceAttributes(query: string): Promise<string>;


  height(query: Query): Promise<Response>;
  height(query: string): Promise<string>;


  transitAvailable(query: Query): Promise<Response>;
  transitAvailable(query: string): Promise<string>;


  expansion(query: Query): Promise<Response>;
  expansion(query: string): Promise<string>;

  centroid(query: Query): Promise<Response>;
  centroid(query: string): Promise<string>;


  status(query: Query): Promise<Response>;
  status(query: string): Promise<string>;

  tile(query: Query | string): Promise<Buffer>;
}

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


  additionalArgs?: string[];
}


export function getConfig(options?: ConfigOptions): Promise<ValhallaConfig>;


export class GraphId {
  /** Default constructor (creates invalid GraphId) */
  constructor();
  /** Construct from raw numeric value */
  constructor(value: number);
  /** Construct from string "level/tileid/id" */
  constructor(value: string);
  /** Construct from components */
  constructor(tileid: number, level: number, id: number);

  /** Raw numeric value */
  readonly value: number;

  /** Get the tile ID */
  tileid(): number;
  /** Get the hierarchy level */
  level(): number;
  /** Get the identifier within the level */
  id(): number;
  /** Check if this GraphId is valid */
  is_valid(): boolean;
  /** Get a GraphId with only tileid and level (id zeroed) */
  tile_base(): GraphId;
  /** Get the 32-bit tile value (level + tileid) */
  tile_value(): number;
  /** Create a new GraphId with the id advanced by offset */
  add(offset: number): GraphId;
  /** Check equality with another GraphId */
  equals(other: GraphId): boolean;
  /** String representation: "level/tileid/id" */
  toString(): string;
  /** JSON representation */
  toJSON(): { level: number; tileid: number; id: number; value: number };
}

/**
 * Get the base (lower-left corner) lon/lat of a tile.
 * @param graphId - A GraphId identifying the tile
 * @returns [lon, lat] coordinate pair of the tile's base corner
 */
export function getTileBaseLonLat(graphId: GraphId): [number, number];

/**
 * Get the tile GraphId for a given coordinate and hierarchy level.
 * @param level - Hierarchy level
 * @param coord - [lon, lat] coordinate pair
 * @returns The tile-base GraphId containing the coordinate
 */
export function getTileIdFromLonLat(level: number, coord: [number, number]): GraphId;

/**
 * Get all tile GraphIds whose tiles intersect a bounding box.
 * @param minx - Minimum longitude
 * @param miny - Minimum latitude
 * @param maxx - Maximum longitude
 * @param maxy - Maximum latitude
 * @param levels - Optional array of hierarchy levels (defaults to [0, 1, 2])
 * @returns Array of tile-base GraphIds
 */
export function getTileIdsFromBbox(minx: number, miny: number, maxx: number, maxy: number, levels?: number[]): GraphId[];

/**
 * Get all tile GraphIds whose tiles intersect or are inside a polygon ring.
 * @param ringCoords - Array of [lon, lat] coordinate pairs forming a polygon ring
 * @param levels - Optional array of hierarchy levels (defaults to [0, 1, 2])
 * @returns Array of tile-base GraphIds
 */
export function getTileIdsFromRing(ringCoords: [number, number][], levels?: number[]): GraphId[];

export const VALHALLA_VERSION: string;
