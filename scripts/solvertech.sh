#!/usr/bin/env bash
echo "Running custom SolverTech scripts..."
echo "Linking NFS to /custom_files"
cd /
latest=1
echo "Using Volume prefix: $volume_prefix"

volumes=`find /mnt -type d -name ${volume_prefix}*`
for volume in $volumes
do
        file=$volume/.timestamp
        echo "Found candidate $file"
        if [ -f "$file" ]; then
                candidate=`stat -c %Y $file`
                if [ $candidate -gt $latest ]; then
                        latest=$candidate
                        selected=$volume
                        echo "$selected is latest so far..."
                fi
        fi
done
echo "Final selection: $selected"
ulimit -a
echo "Current vm.max_map_count:"
sudo sysctl vm.max_map_count
sudo rm -rf /custom_files
sudo ln -sf $selected /custom_files
cd /custom_files
file_name=/custom_files/valhalla.json
if [[ -e "$file_name" ]]; then
  # Check if the file is empty
  if [[ -s "$file_name" ]]; then
    echo "valhalla.json config is ok."
  else
    echo "The file '$file_name' is empty. Restoring it from backup"
	sudo cp /mnt/valhalla.json /custom_files
  fi
else
  echo "The file '$file_name' doesn't exists. Restoring it from backup"
  sudo cp /mnt/valhalla.json /custom_files
fi
echo "Starting valhalla"
/valhalla/scripts/run.sh build_tiles