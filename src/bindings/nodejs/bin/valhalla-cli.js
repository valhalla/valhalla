#!/usr/bin/env node

const { spawn } = require('child_process');
const path = require('path');
const fs = require('fs').promises;
const fsSync = require('fs');
const { getBinaryDir } = require('../lib/binary-path');

const binaryDir = getBinaryDir(path.join(__dirname, '..'));
const packageRootDir = path.join(__dirname, '..');

 

async function fileExists(filePath) {
    try {
        await fs.access(filePath);
        return true;
    } catch {
        return false;
    }
}

async function getAvailableCommands() {
    async function listExecutables(dir) {
        try {
            const files = await fs.readdir(dir);
            const commands = [];
            for (const file of files) {
                const filePath = path.join(dir, file);
                try {
                    const stats = await fs.stat(filePath);
                    if (!stats.isFile()) {
                        continue;
                    }
                    if (file.endsWith('.node')) {
                        continue;
                    }
                    if (!file.startsWith('valhalla_')) {
                        continue;
                    }
                    commands.push(file);
                } catch {
                    continue;
                }
            }
            return commands;
        } catch {
            return [];
        }
    }

    const binCmds = await listExecutables(binaryDir);
    const rootCmds = await listExecutables(packageRootDir);
    return [...binCmds, ...rootCmds];
}

async function printHelp() {
    const packageJson = require('../package.json');
    const commands = await getAvailableCommands();
    
    // Get max command length for alignment
    let maxLen = 0;
    for (const cmd of commands) {
        if (cmd.length > maxLen) {
            maxLen = cmd.length;
        }
    }
    
    console.log(`Valhalla Node.js package v${packageJson.version}\n`);
    console.log('valhalla CLI provides access to Valhalla\'s executables. Arguments are passed through as-is.\n');
    console.log('Usage: valhalla <command> [args...]\n');
    console.log('Options:');
    console.log('  --help, -h     Show this help message');
    console.log('  --version, -v  Show package version');
    console.log('  print_bin_path Print the absolute path to the binaries directory\n');
    
    if (commands.length > 0) {
        console.log(`Available commands (in ${binaryDir}):`);
        for (const cmd of commands) {
            // Show shorthand version without valhalla_ prefix for convenience
            const displayName = cmd.startsWith('valhalla_') ? cmd.substring(9) : cmd;
            const padding = ' '.repeat(Math.max(0, maxLen - cmd.length));
            console.log(`  ${cmd}${padding}  - Run 'valhalla ${displayName} --h' for command help`);
        }
    } else {
        console.log('No binaries found. Make sure the package is properly installed.');
    }
    
    console.log('\nNote: You can use commands with or without the "valhalla_" prefix.');
    console.log('\nExamples:');
    console.log('  valhalla build_tiles -c config.json data.osm.pbf');
    console.log('  valhalla valhalla_build_tiles -c config.json data.osm.pbf  # Full name also works');
}

async function findBinaryIn(dir, command) {
    let binaryPath = path.join(dir, command);
    if (await fileExists(binaryPath)) {
        return binaryPath;
    }
    if (!command.startsWith('valhalla_')) {
        const prefixedCommand = `valhalla_${command}`;
        binaryPath = path.join(dir, prefixedCommand);
        if (await fileExists(binaryPath)) {
            return binaryPath;
        }
    }
    return null;
}

async function findBinary(command) {
    const binaryPath = await findBinaryIn(binaryDir, command);
    if (binaryPath) { return binaryPath; }
    return await findBinaryIn(packageRootDir, command);
}

async function runCommand(command, args) {
    const binaryPath = await findBinary(command);
    
    if (!binaryPath) {
        console.error(`Error: Command '${command}' not found`);
        console.error(`Tried: ${command}${!command.startsWith('valhalla_') ? ` and valhalla_${command}` : ''}`);
        console.error(`Searched in: ${binaryDir} and ${packageRootDir}`);
        console.error(`Run 'valhalla --help' to see available commands.`);
        process.exit(1);
    }
    
    // Spawn the binary
    const proc = spawn(binaryPath, args, {
        stdio: 'inherit', // Pass through stdin, stdout, stderr
        env: process.env
    });
    
    proc.on('error', (err) => {
        console.error(`Error running ${command}:`, err.message);
        process.exit(1);
    });
    
    proc.on('close', (code) => {
        process.exit(code || 0);
    });
}

async function main() {
    const args = process.argv.slice(2);
    
    if (args.length === 0) {
        await printHelp();
        process.exit(0);
    }
    
    const command = args[0];
    const commandArgs = args.slice(1);
    
    if (command === '--help' || command === '-h') {
        await printHelp();
        process.exit(0);
    }
    
    if (command === '--version' || command === '-v') {
        const packageJson = require('../package.json');
        console.log(packageJson.version);
        process.exit(0);
    }
    
    if (command === 'print_bin_path') {
        console.log(binaryDir);
        process.exit(0);
    }
    
    await runCommand(command, commandArgs);
}

main().catch(err => {
    console.error('Error:', err.message);
    process.exit(1);
});

