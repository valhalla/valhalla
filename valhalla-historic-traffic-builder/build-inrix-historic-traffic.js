import fs from 'fs';
import { fork } from 'child_process';
import path from 'path';
import { fileURLToPath } from 'url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const start = Date.now();

// Get all CSV files
const files = fs.readdirSync('./data/inrix').filter(f => f.endsWith('.csv'));
console.log(`Found ${files.length} files to process`);

// Distribute files across processes
const numProcesses = 6;
const processesWork = [];

for (let i = 0; i < numProcesses; i++) {
  const startIndex = Math.floor((i * files.length) / numProcesses);
  const endIndex = Math.floor(((i + 1) * files.length) / numProcesses);
  const work = files.slice(startIndex, endIndex);
  processesWork.push(work);
}

console.log('File distribution:');
processesWork.forEach((work, i) => {
  console.log(`  Process ${i}: ${work.length} files (${work.join(', ')})`);
});

// Fork process function
const forkProcess = (processIndex, work) => new Promise((resolve, reject) => {
  const start = Date.now();
  const workerPath = path.join(__dirname, './InrixRegionWorker.js');
  console.log(`Starting process ${processIndex} with work ${work.slice(0, 2)} of length ${work.length}.`);

  const child = fork(workerPath, [processIndex, ...work]);

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
    // Fork processes for each work batch
    const downloadPromises = processesWork.map((work, index) => forkProcess(index, work));

    // Wait for all processes to complete
    await Promise.all(downloadPromises);

    console.log('Done. Completed in', (Date.now() - start) / 1000, 'seconds');
  } catch (error) {
    console.error('An error occurred:', error);
  }
}

// Run the main function
main();
