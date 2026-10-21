# Dataset acquisition

The current acquisition/merge workflow is documented in the repository-root `train_colab.ipynb` notebook. Open it in Google Colab and execute it from top to bottom. It requests a Roboflow API key, downloads the public fastener datasets, maps the annotations to `washer`, `nut`, and `bolt`, and prepares `datasets/merged/data.yaml`.

Never commit the API key. Use Colab Secrets or a runtime-only input. Do not commit the downloaded image/label directories by default; `.gitignore` intentionally excludes them.

## Identified versioned source exports

- `yellow_silver`, version 4: https://universe.roboflow.com/jingjing-huang/yellow_silver/dataset/4
- `nutsnbolts`, version 5: https://universe.roboflow.com/tktk-hodc3/nutsnbolts/dataset/5

These two entries were identified from local dataset metadata. The notebook states that 12 datasets are downloaded in total, so this list is not yet the complete source manifest. Before claiming exact training reproduction, enumerate all dataset projects and export versions from the notebook/downloader and record the licence, attribution, and a SHA-256 checksum for each downloadable archive or exported manifest.

The merged `data.yaml` is generated output. Make its dataset root relocatable (relative to the generated dataset directory) rather than keeping an absolute path tied to one user's machine. Preserve class names and the train/validation/test split. Dataset files are not automatically redistributable just because they are publicly downloadable; review each dataset's terms before sharing it.
