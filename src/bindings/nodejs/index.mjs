import { createRequire } from 'module';

const require = createRequire(import.meta.url);
const cjsModule = require('./index.js');

export const Actor = cjsModule.Actor;
export const GraphId = cjsModule.GraphId;
export const getConfig = cjsModule.getConfig;
export const getTileIdFromLonLat = cjsModule.getTileIdFromLonLat;
export const getTileIdsFromBbox = cjsModule.getTileIdsFromBbox;
export const getTileIdsFromRing = cjsModule.getTileIdsFromRing;
export const VALHALLA_VERSION = cjsModule.VALHALLA_VERSION;

export default cjsModule;

