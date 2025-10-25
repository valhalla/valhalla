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


export const VALHALLA_VERSION: string;
