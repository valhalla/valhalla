import { createRequire } from 'module';

const require = createRequire(import.meta.url);
const cjsModule = require('./index.js');

export const Actor = cjsModule.Actor;
export const getConfig = cjsModule.getConfig;
export const VALHALLA_VERSION = cjsModule.VALHALLA_VERSION;

export default cjsModule;

