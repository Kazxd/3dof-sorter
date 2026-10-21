# Training the detector (one click, no own dataset needed)

[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/Kazxd/3dof-sorter/blob/main/train_colab.ipynb)

1. Click **Open in Colab** above and sign in with any Google account.
2. **Runtime → Change runtime type → T4 GPU → Save.**
3. **Runtime → Run all.**
4. Paste a free Roboflow API key when asked (roboflow.com → sign up → Settings → API Keys).
5. Wait about an hour with the tab open. `best.pt` downloads by itself at the end.
6. Put `best.pt` in `host/models/`, then commit and push it so everyone has it:

   ```bash
   git add host/models/best.pt
   git commit -m "Add trained detector"
   git push
   ```

## What it does

* Downloads 12 public datasets, about 17k images. Each was checked by hand for a
  top-down view of loose parts like our camera's. The list is at the top of
  `host/get_model.py`.
* Merges them into three classes (washer, nut, bolt) and drops everything else.
* Fine-tunes YOLOv8n for 40 epochs on Google's GPU.

No photos of our own setup are needed. If the model misses parts on our mat later,
the sorter falls back to the colour + shape detector by itself, and our own photos can
be added at any time with `collect_images.py` and a rerun.

## If something goes wrong

| Problem | Fix |
|---|---|
| "No GPU" in the first cell | Step 2 wasn't done; change the runtime and Run all again |
| A dataset prints `[error]` | Ignore it; the others still train. Training needs at least one to work. |
| Colab disconnected mid-training | Run the training cell again |
| Download didn't start | Left sidebar → Files → `3dof-sorter/host/models/best.pt` → ⋮ → Download |
