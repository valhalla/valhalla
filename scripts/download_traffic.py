#!/usr/bin/env python3
from azure.storage.blob import BlockBlobService  # v2.1.0
import os
import logging
import argparse


def get_args():
    """
    Description: 
        get the arguments of python modules

    Return: 
        arguments
    """
    parser = argparse.ArgumentParser(
        description="Downloads Traffic messages from Azure Blob Storage. ")

    parser.add_argument("--account_name", type=str,
                        help="Account Name of Azure Azure Blob Storage", required=True)
    parser.add_argument("--account_key", type=str,
                        help="Account Key of Azure Azure Blob Storage", required=True)
    parser.add_argument("--container_name", type=str,
                        help="Container Name of Azure Azure Blob Storage", required=True)
    parser.add_argument("--out_path", type=str,
                        help="Path to Store the Traffic Messages", required=True)
    args = parser.parse_args()
    return args


def download_container_to_path(account_name, account_key, container_name, out_path):
    """
    Description: 
        download a azure storage container to local path 

    Parameters: 
        account_name, account_key: account_name and key for azure storage
        container_name: azure container name (string)
        path: local path (string)

    Return:
        duration of processing time in seconds           
    """
    try:
        blob_client = BlockBlobService(account_name, account_key)
        generator = blob_client.list_blobs(container_name)
        for blob in generator:
            blob_client.get_blob_to_path(
                container_name, blob.name, os.path.join(out_path, blob.name))
            logging.info("Downloaded " + blob.name)
    except:
        logging.info("Could not download " + blob.name)


def main():
    args = get_args()
    if not os.path.exists(args.out_path):
        os.makedirs(args.out_path)
    # logging.basicConfig(filename=os.path.join(args.out_path, 'download_traffic.log'),
    #                     format='%(asctime)s %(levelname)-8s %(message)s', level=logging.INFO,  datefmt='%Y-%m-%d %H:%M:%S')

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s.%(msecs)06d [%(levelname)s] %(message)s",
        handlers=[
            # logging.FileHandler(os.path.join(args.out_path, 'download_traffic.log'),
            logging.StreamHandler()
        ],  datefmt='%Y/%m/%d %H:%M:%S')

    logging.getLogger("azure.storage.common.storageclient").setLevel(
        logging.WARNING)

    download_container_to_path(
        args.account_name, args.account_key, args.container_name,  args.out_path)


if __name__ == '__main__':
    main()
