export const toGraphIdString = (graphId) => {
  const value = BigInt(graphId);
  const level = Number(value & 0x7n);
  const tileId = Number((value & 0x1fffff8n) >> 3n);
  const id = Number((value & 0x3ffffe000000n) >> 25n);
  return `${level}/${tileId}/${id}`;
}

/**
 * creates a file suffix path from a graph ID, similar to Valhalla's GraphTile::FileSuffix
 * returns formatted path like "0/000/000/001.csv"
 */
export const toTileIdString = (graphId) => {
  const value = BigInt(graphId);
  const level = Number(value & 0x7n);
  const tileId = Number((value & 0x1fffff8n) >> 3n);

  // maximum tile ID for a level depends on the hierarchy
  // in Valhalla this comes from TileHierarchy but we'll simplify
  // and just use the maximum possible value for 22 bits
  const maxId = 0x1fffff; // 22 bits = 2,097,151

  // calculate how many digits in max tile id
  let maxLength = Math.max(1, Math.floor(Math.log10(maxId)) + 1);

  // adjust to multiple of 3 (for group formatting)
  const remainder = maxLength % 3;
  if (remainder !== 0) {
    maxLength += 3 - remainder;
  }

  // convert tile ID to a padded string with separators
  let tileIdStr = tileId.toString().padStart(maxLength, '0');

  // add separators every 3 digits
  let formattedPath = '';
  for (let i = 0; i < tileIdStr.length; i += 3) {
    formattedPath += '/' + tileIdStr.substring(i, i + 3);
  }

  // combine level, formatted tile ID path, and suffix
  return `${level}${formattedPath}`;
};
