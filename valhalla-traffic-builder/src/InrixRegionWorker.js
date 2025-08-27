// This is a helper file used for running a Inrix region download
// as a standalone process
import InrixTrafficProvider from "./InrixTrafficProvider.js";

const work = process.argv[6].split(',');
const processIndex = process.argv[1];

if (!work) {
  console.error("Work argument is required");
  process.exit(1);
}

try {
  console.log(`Starting process ${processIndex} with work ${work.slice(0, 20)} of length ${work.length}.`)
  await InrixTrafficProvider.downloadBatch(work, process.argv[3], process.argv[4], process.argv[5]);
  process.exit(0);
} catch (error) {
  console.error(`Error processing region ${processIndex}:`, error);
  process.exit(1);
}
