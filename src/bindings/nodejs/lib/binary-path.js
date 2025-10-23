const path = require('path');
const os = require('os');


function getBinaryDir(baseDir) {
    const platform = os.platform();
    const arch = os.arch();

    const SUPPORTED = [
        'darwin/arm64',
        'linux/x86_64',
        'linux/arm64',
    ];

    if (!SUPPORTED.includes(`${platform}/${arch}`)) {
        throw new Error(`Unsupported platform and architecture: ${platform}/${arch}`);
    }


    return path.join(baseDir, platformDir, archDir);
}

module.exports = {
    getBinaryDir
};

