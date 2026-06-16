# Publishing a dataset to the cloud (runbook)

## Lesson first — why NOT Google Drive

Google Drive is the wrong tool for distributing a large/popular dataset. A shared Drive file enforces
a **per-file download quota** and **per-user copy limits** — a popular file returns
`downloadQuotaExceeded` (~24 h reset) and copies hit `userRateLimitExceeded`. We hit exactly this with
RH20T (see [the RH20T checkpoint]). For a dataset others will pull, use **Google Cloud Storage (GCS)**
or **Hugging Face**, not Drive.

> **Rule of thumb:** GCS (requester-pays) **or** Hugging Face. Private + small → Drive with your own
> OAuth client. **Always publish `SHA256SUMS` + a `manifest.json`** for provenance/integrity.

## For wide-robot (recommendation)

What actually needs the cloud here is the **raw capture media** — `recordings/raw_videos/**/*.mp4`
(~5.1 GB) is gitignored and *cannot* live in git. The **derived JSON** (tracks/rollouts/calibrations/
`verdicts_*.json`) is the small, auditable record that stays in git (and per the open repo-weight
question, future phases may slim git to verdicts + calibrations + a fixture subset and move the bulk
to the same cloud dataset).

- **Recommended: Hugging Face Hub** — best fit for a robotics/ML dataset others reuse: no quota walls,
  versioned, discoverable, free for public datasets. Ship the raw videos + `recordings/manifest.json`
  (already carries each clip's SHA-256) + a top-level `SHA256SUMS`.
- **Alternative: GCS requester-pays** — if you want it in your own cloud and don't want to eat egress.

Either way, keep `recordings/manifest.json` + `datasets/.../INGESTION_RESULTS.md` in git as the index
that points at the published media by SHA-256, so the derived evidence and the raw media stay bound.

## Option A — Google Cloud Storage bucket (best for "our own cloud, others download")

`gcloud` is authed as `podflyy@gmail.com` (owns the GCP projects). GCS needs billing enabled on the
project (small data ≈ cents; 5 GB free tier).

```bash
gcloud config set project <PROJECT_ID>
gcloud services enable storage.googleapis.com
# Bucket names are GLOBALLY unique; pick a region near your users:
gcloud storage buckets create gs://<BUCKET> --location=US --uniform-bucket-level-access
# Upload (parallel + resumable; works off a stream too):
gcloud storage cp -r ./mydataset gs://<BUCKET>/mydataset
# Integrity: ship checksums + a manifest alongside the data
( cd mydataset && find . -type f -exec sha256sum {} + ) > SHA256SUMS
gcloud storage cp SHA256SUMS manifest.json gs://<BUCKET>/mydataset/
```

Make it accessible — pick ONE:

```bash
# (a) Public read — anyone downloads, no auth, plain HTTPS URLs:
gcloud storage buckets add-iam-policy-binding gs://<BUCKET> \
  --member=allUsers --role=roles/storage.objectViewer
#   -> https://storage.googleapis.com/<BUCKET>/mydataset/<file>

# (b) Signed URLs — time-limited, no public exposure:
gcloud storage sign-url --duration=7d gs://<BUCKET>/mydataset/<file>

# (c) Requester-pays — downloaders cover egress so YOU don't get the bill:
gcloud storage buckets update gs://<BUCKET> --requester-pays
```

**Cost gotcha:** storage ≈ `$0.02/GB·mo`; egress ≈ `$0.12/GB` to the internet — for a popular dataset
use **(c) requester-pays** or front it with a CDN. Users download with
`gcloud storage cp gs://<BUCKET>/... .` or the HTTPS URL.

## Option B — Hugging Face Hub (best for an ML/robotics dataset others reuse)

No quota walls, versioned, discoverable, free for public datasets:

```bash
pip install -U huggingface_hub && huggingface-cli login
huggingface-cli upload <user>/<dataset> ./mydataset --repo-type=dataset
# others: huggingface-cli download <user>/<dataset> --repo-type=dataset
```

## Option C — Drive (only if you must; small/private)

Use your **own** OAuth client (never rclone's shared default — it's globally rate-limited): GCP Console
→ enable Drive API → Credentials → OAuth client ID → Desktop app → add yourself as a test user. Then:

```bash
# browser-authorize on first run:
rclone config create mydrive drive scope drive client_id <ID> client_secret <SECRET>
rclone copy ./mydataset mydrive:mydataset -P
```

Share the folder via link — but warn users: popular/large files will hit `downloadQuotaExceeded`; the
only reliable consumer workaround is "make a copy into your own Drive, then download the copy."

---

[the RH20T checkpoint]: ../datasets/rh20t_object_inside_container_v0/
