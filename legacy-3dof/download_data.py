import kagglehub

# Download latest version of the dataset
path = kagglehub.dataset_download("ishantvinaik/synthetic-nuts-and-bolts-data")
print("========================================")
print("SUCCESS! DATASET DOWNLOADED TO:")
print(path)
print("========================================")