// Usage node quadkey_generator.js

function quadkeyToTile(quadkey) {
  let x = 0;
  let y = 0;
  const z = quadkey.length;
  for (let i = 0; i < z; i++) {
    const mask = 1 << (z - i - 1);
    const digit = parseInt(quadkey[i]);
    if (digit & 1) x |= mask;
    if (digit & 2) y |= mask;
  }
  return { x, y, z };
}

function tileToQuadkey(x, y, z) {
  let quadkey = '';
  for (let i = z - 1; i >= 0; i--) {
    let digit = 0;
    const mask = 1 << i;
    if (x & mask) digit += 1;
    if (y & mask) digit += 2;
    quadkey += digit;
  }
  return quadkey;
}

function getQuadkeysInBox(q1, q2, q3, q4) {
  const tiles = [quadkeyToTile(q1), quadkeyToTile(q2), quadkeyToTile(q3), quadkeyToTile(q4)];
  const z = tiles[0].z;

  const minX = Math.min(...tiles.map(t => t.x));
  const maxX = Math.max(...tiles.map(t => t.x));
  const minY = Math.min(...tiles.map(t => t.y));
  const maxY = Math.max(...tiles.map(t => t.y));

  const result = [];
  for (let x = minX; x <= maxX; x++) {
    for (let y = minY; y <= maxY; y++) {
      result.push(tileToQuadkey(x, y, z));
    }
  }

  return result;
}

// USA Quadkeys
// Bulk of USA
const bulkUSAQuadKeys = getQuadkeysInBox(
  "0212330", // NW
  "0230132", // SW
  "0302231", // NE
  "0320033"  // SE
);

const northUSAQuadKeys = getQuadkeysInBox(
  "0212122",
  "0212303",
  "0213132",
  "0213312",
);

const californiaQuadkeys = getQuadkeysInBox(
  "0212320", // NW
  "0230120", // SW
  "0212321", // NE
  "0230121"  // SE
);


const southQuadKeys = getQuadkeysInBox(
  "0230311",
  "0230311",
  "0320211",
  "0320211"
);

const pacificNorthWestQuadKeys = getQuadkeysInBox(
  "0212211",
  "0212211",
  "0230011",
  "0230011"
)

const neQuadKeys = getQuadkeysInBox(
  "0302320", // NW
  "0302322", // SW
  "0302331", // NE
  "0302333"  // SE
);

const eastQuadKeys = getQuadkeysInBox("0320100", "0320120", "0320101", "0320121");

const texasQuadKeys = getQuadkeysInBox("0231212", "0231212", "0320202", "0320202");

let quadkeys = [...bulkUSAQuadKeys, ...northUSAQuadKeys, ...californiaQuadkeys, ...southQuadKeys, ...pacificNorthWestQuadKeys, ...neQuadKeys, ...eastQuadKeys, ...texasQuadKeys];

// https://labs.mapbox.com/what-the-tile/ very useful for finding quadkeys

quadkeys.push("0230123");  // LA
quadkeys.push("0320122") // South Carolina
quadkeys.push(...["0231320", "0231321"]); // Last of Texas
quadkeys.push(...["0213311", "0302200", "0213313", "0302202", "0302203", "0302212"]) // Michigan
quadkeys.push("0320110"); // long island
quadkeys.push(...["0302312", "0302313"]); // Maine (there's two more but there's no way there's any data there)
quadkeys.push(...["0320212", "0320213", "0320230", "0320231"]) // Florida


quadkeys.push("0201210");  // Anchorage
quadkeys.push("0201013");   // Fairbanks
quadkeys.push("0210220");   // Juneau
quadkeys.push("0323002"); // Puerto Rico

quadkeys.push(...["0220333", "0222111", "0223000", "0223002"]); // Hawaii


quadkeys.push(...['0212113', '0212131', '0212133', '0213030']); // Calgary -> Edmonton, Saskatoon
quadkeys.push(...getQuadkeysInBox('0302201', '0302203', '0303200', '0303202')); // some of Ontario (that wasn't already included above)
quadkeys.push('0303023') // PEI
quadkeys.push(...['0303301']) // St John
quadkeys.push(...['0303220', '0303221', '0303203', '0303212']); // Nova Scotia


quadkeys.push(...['0230310', '0230313', '0230331', '0231220', '0231222', '0231223']); // Baja Peninsula, Mexico
quadkeys.push(...['0231202', '0231203']);
quadkeys.push(...getQuadkeysInBox('0231221', '0231223', '0231321', '0231323'))
quadkeys.push(...getQuadkeysInBox('0233010', '0233101', '0233012' ,'0233103'))
quadkeys.push(...['0233121', '0233130', '0233112', '0233113', '0233111', '0322000', '0322001']) // remaining mexico quadkeys


// Turkey
quadkeys.push(...getQuadkeysInBox('1203223', '1221003', '1203333', '1221113'));
quadkeys.push(...['1221030', '1221031', '1221120'])



function shuffle(array) {
  for (let i = array.length - 1; i > 0; i--) {
    const j = Math.floor(Math.random() * (i + 1));
    [array[i], array[j]] = [array[j], array[i]]; // swap
  }
  return array;
}

// Remove duplicates
quadkeys = [...new Set(quadkeys)];

shuffle(quadkeys); // Shuffle to distribute population density


console.log(quadkeys.length);
console.log(JSON.stringify(quadkeys, null, 2));



