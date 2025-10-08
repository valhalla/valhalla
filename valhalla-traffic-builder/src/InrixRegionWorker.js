// This is a helper file used for running a Inrix region download
// as a standalone process
import InrixTrafficProvider from "./InrixTrafficProvider.js";

const trafficWork = process.argv[6].split(',');
const incidentWork = process.argv[7].split('|');
const processIndex = process.argv[1];

if (!trafficWork) {
  console.error("Traffic work argument is required");
  process.exit(1);
}

try {
  console.log(`Starting process ${processIndex} with trafficWork ${trafficWork.slice(0, 20)} of length ${trafficWork.length}.`)
  await InrixTrafficProvider.downloadBatch(trafficWork, process.argv[3], process.argv[4], process.argv[5]);
  await InrixTrafficProvider.downloadBatchIncidents(incidentWork, process.argv[3], process.argv[4], process.argv[5]);
  process.exit(0);
} catch (error) {
  console.error(`Error processing region ${processIndex}:`, error);
  process.exit(1);
}
