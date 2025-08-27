import path from 'path';
import { fork } from 'child_process';
import { fileURLToPath } from 'url';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

const start = Date.now();


function divideArrayIntoChunks(array, numChunks) {
  const chunkSize = Math.floor(array.length / numChunks);
  const remainder = array.length % numChunks;
  const result = [];

  let start = 0;
  for (let i = 0; i < numChunks; i++) {
      const end = start + chunkSize + (i < remainder ? 1 : 0);
      result.push(array.slice(start, end));
      start = end;
  }

  return result;
}

// inrix is bounded by Microsoft quadkey: https://learn.microsoft.com/en-us/bingmaps/articles/bing-maps-tile-system?redirectedfrom=MSDN
const args = process.argv.slice(2);

const processes = parseInt(args[0].split('=')[1]);
const valhallaPath = args[1].split('=')[1]
const trafficPath = args[2].split('=')[1];
const inputPath = args[3].split('=')[1];


// const quadkeys = ['0302231', '0302233', '0302320', '0302322', '0302321', '0302323', '0320101', '0320110' ]; // New York State Quadkeys


// Generated from quadkey_generator.js
const quadkeys = [
  "0231310",
  "0231133",
  "0231232",
  "0231322",
  "0213311",
  "0230121",
  "0231110",
  "0230101",
  "0212213",
  "0302202",
  "0233113",
  "0320201",
  "0231313",
  "0231213",
  "0230131",
  "0231302",
  "0302203",
  "0213211",
  "0220333",
  "0231012",
  "0212113",
  "0210220",
  "0230110",
  "0231300",
  "0302223",
  "0231100",
  "0213032",
  "0320031",
  "0233013",
  "0231321",
  "0320023",
  "0212313",
  "1221103",
  "1221011",
  "0212320",
  "0302233",
  "0302331",
  "0213202",
  "0233111",
  "0230133",
  "1221100",
  "0233102",
  "0320103",
  "0231132",
  "0302213",
  "0213232",
  "0212132",
  "0302332",
  "0302322",
  "0212332",
  "0230112",
  "0212231",
  "0302201",
  "0302321",
  "0320033",
  "1221110",
  "0302302",
  "0212311",
  "0231130",
  "1221030",
  "0213213",
  "0303301",
  "0231120",
  "0222111",
  "0233010",
  "1221001",
  "0302231",
  "0213222",
  "0302232",
  "0231131",
  "1221012",
  "0231303",
  "0231223",
  "0213221",
  "0320210",
  "0213320",
  "0230111",
  "0231001",
  "0231220",
  "0302311",
  "0233103",
  "0231123",
  "0212303",
  "0213302",
  "0231233",
  "0302222",
  "0303200",
  "0231031",
  "0233101",
  "0231101",
  "0231011",
  "0212331",
  "0322001",
  "0323002",
  "0212323",
  "0230130",
  "0233012",
  "0213210",
  "0212133",
  "0320213",
  "0320202",
  "0320230",
  "0231003",
  "0320000",
  "0231002",
  "0212302",
  "0212131",
  "0213231",
  "0213330",
  "0231103",
  "0320231",
  "0302333",
  "1221003",
  "0320012",
  "0213023",
  "0322000",
  "0303203",
  "0233121",
  "0231122",
  "0213033",
  "0213203",
  "0230132",
  "0320001",
  "0231230",
  "0231210",
  "0302301",
  "0201210",
  "1203333",
  "0231203",
  "1203332",
  "0302230",
  "0320030",
  "1221013",
  "0231030",
  "0320212",
  "0231200",
  "0231022",
  "0320021",
  "0213303",
  "0212333",
  "1203232",
  "0213233",
  "0231121",
  "0213322",
  "0320110",
  "0213331",
  "1221102",
  "0213132",
  "0233112",
  "0213301",
  "0212211",
  "0230313",
  "0302323",
  "0231113",
  "0212322",
  "0212312",
  "0302212",
  "0213313",
  "0320211",
  "0320101",
  "0231032",
  "0320032",
  "0233100",
  "0213123",
  "0231102",
  "0320020",
  "0302303",
  "0212330",
  "0320013",
  "0320200",
  "0213030",
  "0231020",
  "0303212",
  "0303221",
  "0213223",
  "0320011",
  "1221111",
  "0231231",
  "0212310",
  "0231021",
  "0213300",
  "0212301",
  "0302221",
  "0302200",
  "0302211",
  "0231312",
  "0212321",
  "0213230",
  "0231112",
  "1221112",
  "0231201",
  "0233130",
  "0230123",
  "0231111",
  "0230331",
  "0230120",
  "0212233",
  "1221010",
  "1221113",
  "0320122",
  "0230103",
  "0302312",
  "0320100",
  "0213333",
  "0213022",
  "0230311",
  "0320121",
  "0302300",
  "0213312",
  "0231202",
  "0320120",
  "0231211",
  "0320022",
  "0230011",
  "1221120",
  "0231000",
  "0230113",
  "0212123",
  "0213332",
  "0212122",
  "0302210",
  "0231222",
  "0213122",
  "0320102",
  "1203233",
  "0320002",
  "1203322",
  "0231301",
  "0302330",
  "0231221",
  "0233011",
  "0231212",
  "0320010",
  "0302313",
  "0230102",
  "0231311",
  "1203223",
  "1221031",
  "0231010",
  "0231320",
  "0230310",
  "0212300",
  "0231323",
  "0303202",
  "0223000",
  "0302320",
  "0213321",
  "1221101",
  "0213212",
  "0201013",
  "1203323",
  "0303023",
  "0213200",
  "0231013",
  "0213201",
  "0230100",
  "0213310",
  "0320003",
  "0303220",
  "0302220",
  "0213220",
  "0231033",
  "0223002",
  "0302310",
  "0213323",
  "0231023"
]

console.log(`Ingesting ${quadkeys.length} quadkeys at date: ${new Date().toISOString()}.`);

const processesWork = divideArrayIntoChunks(quadkeys, processes);

// create and manage a forked process for each region
const forkProcess = (processIndex, work) => new Promise((resolve, reject) => {
  const start = Date.now();
  const workerPath = path.join(__dirname, './src/InrixRegionWorker.js');
  console.log(`Starting process ${processIndex} with work ${work.slice(0, 20)} of length ${work.length}.`)
  const child = fork(workerPath, [processIndex, valhallaPath, trafficPath, inputPath, work]);

  child.on('exit', (code) => {
    if (code === 0) {
      console.log(`${processIndex} done. Completed in ${(Date.now() - start) / 1000} seconds.`);
      resolve();
    } else {
      reject(new Error(`${processIndex} failed with code ${code}`));
    }
  });

  child.on('error', (err) => {
    reject(new Error(`Error in process for ${processIndex}: ${err.message}`));
  });
});

async function main() {
  try {
    // fork processes for each process
    const downloadPromises = processesWork.map((work, index) => forkProcess(index, work));

    // wait for all processes to complete
    await Promise.all(downloadPromises);

    console.log('Done. Completed in', (Date.now() - start) / 1000, 'seconds');
  } catch (error) {
    console.error('An error occurred:', error);
  }
}

// Run the main function
main();
