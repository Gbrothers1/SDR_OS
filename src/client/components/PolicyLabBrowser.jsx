import React, { useCallback, useEffect, useMemo, useState } from 'react';
import { useGenesis } from '../contexts/GenesisContext';

const INITIAL_CHECKPOINT_LIMIT = 150;
const INITIAL_VIDEO_LIMIT = 80;

function humanSize(bytes) {
  if (!Number.isFinite(bytes) || bytes <= 0) return '0 B';
  const units = ['B', 'KB', 'MB', 'GB', 'TB'];
  const index = Math.min(Math.floor(Math.log(bytes) / Math.log(1024)), units.length - 1);
  const value = bytes / (1024 ** index);
  return `${value >= 100 || index === 0 ? value.toFixed(0) : value.toFixed(1)} ${units[index]}`;
}

function formatDate(isoString) {
  if (!isoString) return 'Unknown';
  const date = new Date(isoString);
  if (Number.isNaN(date.getTime())) return 'Unknown';
  return date.toLocaleString();
}

function policyKey(policy) {
  return policy?.id || policy?.path || `${policy?.source}/${policy?.name}`;
}

// Eval status from run_registry.jsonl (backend attaches policy.validation).
const EVALUATED_STATUSES = ['validated', 'smoke_pass', 'failed'];
const EVAL_LABEL = { validated: 'Validated', smoke_pass: 'Smoke', failed: 'Failed', untested: 'Untested' };
const evalStatusOf = (policy) => policy?.validation?.status || 'untested';

// Editable tag chips. Persists via the node lab API; the sim runner reads the
// same file on every scan, so saved tags show up in the Policy Library panel
// after the next list refresh (onSaved triggers one).
function TagEditor({ policy, onSaved }) {
  const [tags, setTags] = useState(policy.tags || []);
  const [draft, setDraft] = useState('');
  const [saving, setSaving] = useState(false);
  const [error, setError] = useState(null);

  useEffect(() => {
    setTags(policy.tags || []);
    setDraft('');
    setError(null);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [policy.name]);

  const save = async (next) => {
    setSaving(true);
    setError(null);
    try {
      const res = await fetch(`/api/lab/tags/${encodeURIComponent(policy.name)}`, {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ tags: next }),
      });
      const data = await res.json().catch(() => ({}));
      if (!res.ok) throw new Error(data.error || `HTTP ${res.status}`);
      setTags(data.tags || next);
      if (onSaved) onSaved();
    } catch (e) {
      setError(e.message || 'Tag save failed');
    } finally {
      setSaving(false);
    }
  };

  const addTag = () => {
    const t = draft.trim().toLowerCase();
    setDraft('');
    if (!t || tags.includes(t)) return;
    save([...tags, t]);
  };

  return (
    <div className="policy-lab__tags">
      <span className="policy-lab__tags-label">Tags</span>
      {tags.map((tag) => (
        <span key={tag} className="policy-lab__tag-chip">
          {tag}
          <button
            className="policy-lab__tag-remove"
            onClick={() => save(tags.filter((x) => x !== tag))}
            disabled={saving}
            title={`Remove "${tag}"`}
          >
            ×
          </button>
        </span>
      ))}
      <input
        className="policy-lab__input policy-lab__input--tag"
        type="text"
        placeholder="add tag"
        value={draft}
        onChange={(e) => setDraft(e.target.value)}
        onKeyDown={(e) => {
          if (e.key === 'Enter') {
            e.preventDefault();
            addTag();
          }
        }}
        disabled={saving}
      />
      {saving && <span className="policy-lab__tags-saving">saving…</span>}
      {error && <span className="policy-lab__danger">{error}</span>}
    </div>
  );
}

export default function PolicyLabBrowser({ active }) {
  const {
    policyList,
    policyLoadStatus,
    policyLoadError,
    policyCheckpoint,
    genesisConnected,
    listPolicies,
    loadPolicy,
  } = useGenesis();

  const [query, setQuery] = useState('');
  const [obsFilter, setObsFilter] = useState('all');
  const [evalFilter, setEvalFilter] = useState('all');
  const [selectedKey, setSelectedKey] = useState(null);
  const [detail, setDetail] = useState(null);
  const [detailStatus, setDetailStatus] = useState('idle');
  const [detailError, setDetailError] = useState(null);
  const [checkpointQuery, setCheckpointQuery] = useState('');
  const [videoQuery, setVideoQuery] = useState('');
  const [checkpointLimit, setCheckpointLimit] = useState(INITIAL_CHECKPOINT_LIMIT);
  const [videoLimit, setVideoLimit] = useState(INITIAL_VIDEO_LIMIT);
  const [selectedVideoName, setSelectedVideoName] = useState(null);
  const [loadingCheckpoint, setLoadingCheckpoint] = useState(null);

  useEffect(() => {
    if (active && genesisConnected) listPolicies();
  }, [active, genesisConnected, listPolicies]);

  useEffect(() => {
    if (policyLoadStatus === 'loaded' || policyLoadStatus === 'error') {
      setLoadingCheckpoint(null);
      if (policyLoadStatus === 'loaded') listPolicies();
    }
  }, [policyLoadStatus, listPolicies]);

  const policies = useMemo(
    () => policyList.filter((policy) => policy.type === 'directory'),
    [policyList]
  );

  const obsDimensions = useMemo(() => (
    [...new Set(
      policies
        .map((policy) => policy.obs_dim)
        .filter((dimension) => dimension != null)
        .map(Number)
    )].sort((a, b) => a - b)
  ), [policies]);

  const filteredPolicies = useMemo(() => {
    const normalizedQuery = query.trim().toLowerCase();
    return policies.filter((policy) => {
      if (obsFilter === 'unknown' && policy.obs_dim != null) return false;
      if (
        obsFilter !== 'all' &&
        obsFilter !== 'unknown' &&
        Number(policy.obs_dim) !== Number(obsFilter)
      ) {
        return false;
      }
      if (evalFilter === 'evaluated' && !EVALUATED_STATUSES.includes(evalStatusOf(policy))) return false;
      if (evalFilter !== 'all' && evalFilter !== 'evaluated' && evalStatusOf(policy) !== evalFilter) return false;
      if (!normalizedQuery) return true;
      return (
        (policy.name || '').toLowerCase().includes(normalizedQuery) ||
        (policy.source || '').toLowerCase().includes(normalizedQuery) ||
        (policy.path || '').toLowerCase().includes(normalizedQuery)
      );
    });
  }, [policies, obsFilter, evalFilter, query]);

  const evalCounts = useMemo(() => {
    const c = { all: policies.length, evaluated: 0, validated: 0, smoke_pass: 0, failed: 0, untested: 0 };
    policies.forEach((policy) => {
      const s = evalStatusOf(policy);
      c[s] = (c[s] || 0) + 1;
      if (EVALUATED_STATUSES.includes(s)) c.evaluated += 1;
    });
    return c;
  }, [policies]);

  const selectedPolicy = useMemo(
    () => policies.find((policy) => policyKey(policy) === selectedKey) || null,
    [policies, selectedKey]
  );

  useEffect(() => {
    if (!active || filteredPolicies.length === 0) return;
    const selectedIsVisible = filteredPolicies.some(
      (policy) => policyKey(policy) === selectedKey
    );
    if (!selectedIsVisible) {
      const preferred = filteredPolicies.find((policy) => policy.compatible !== false);
      setSelectedKey(policyKey(preferred || filteredPolicies[0]));
    }
  }, [active, filteredPolicies, selectedKey]);

  useEffect(() => {
    if (!active || !selectedPolicy?.source || !selectedPolicy?.name) {
      setDetail(null);
      return undefined;
    }

    const controller = new AbortController();
    setDetail(null);
    setDetailStatus('loading');
    setDetailError(null);
    setCheckpointQuery('');
    setVideoQuery('');
    setCheckpointLimit(INITIAL_CHECKPOINT_LIMIT);
    setVideoLimit(INITIAL_VIDEO_LIMIT);
    setSelectedVideoName(null);

    const source = encodeURIComponent(selectedPolicy.source);
    const name = encodeURIComponent(selectedPolicy.name);
    fetch(`/api/lab/policies/${source}/${name}`, { signal: controller.signal })
      .then(async (response) => {
        const payload = await response.json();
        if (!response.ok) throw new Error(payload.error || `Artifact request failed (${response.status})`);
        return payload;
      })
      .then((payload) => {
        setDetail(payload);
        setSelectedVideoName(payload.videos?.[0]?.name || null);
        setDetailStatus('loaded');
      })
      .catch((error) => {
        if (error.name === 'AbortError') return;
        setDetailStatus('error');
        setDetailError(error.message);
      });

    return () => controller.abort();
  }, [active, selectedPolicy?.source, selectedPolicy?.name]);

  const checkpoints = useMemo(() => {
    const normalizedQuery = checkpointQuery.trim().toLowerCase();
    const items = detail?.checkpoints || [];
    if (!normalizedQuery) return items;
    return items.filter((checkpoint) => (
      checkpoint.name.toLowerCase().includes(normalizedQuery) ||
      String(checkpoint.step ?? '').includes(normalizedQuery)
    ));
  }, [detail, checkpointQuery]);

  const videos = useMemo(() => {
    const normalizedQuery = videoQuery.trim().toLowerCase();
    const items = detail?.videos || [];
    if (!normalizedQuery) return items;
    return items.filter((video) => video.name.toLowerCase().includes(normalizedQuery));
  }, [detail, videoQuery]);

  const selectedVideo = useMemo(
    () => (detail?.videos || []).find((video) => video.name === selectedVideoName) || null,
    [detail, selectedVideoName]
  );

  const handleLoad = useCallback((checkpointName) => {
    if (!selectedPolicy || selectedPolicy.compatible === false) return;
    setLoadingCheckpoint(checkpointName);
    loadPolicy(selectedPolicy.path, checkpointName);
  }, [selectedPolicy, loadPolicy]);

  if (!active) return null;

  return (
    <div className="policy-lab">
      <aside className="policy-lab__library">
        <div className="policy-lab__filters">
          <input
            className="policy-lab__input"
            type="search"
            placeholder="Search policies"
            value={query}
            onChange={(event) => setQuery(event.target.value)}
          />
          <select
            className="policy-lab__select"
            value={obsFilter}
            onChange={(event) => setObsFilter(event.target.value)}
            aria-label="Filter Lab policies by observation dimension"
          >
            <option value="all">All obs dims</option>
            {obsDimensions.map((dimension) => (
              <option key={dimension} value={dimension}>{dimension} obs</option>
            ))}
            <option value="unknown">Unknown obs</option>
          </select>
          <select
            className="policy-lab__select"
            value={evalFilter}
            onChange={(event) => setEvalFilter(event.target.value)}
            aria-label="Filter Lab policies by evaluation status"
          >
            <option value="all">All eval ({evalCounts.all})</option>
            <option value="evaluated">Evaluated ({evalCounts.evaluated})</option>
            <option value="untested">Not evaluated ({evalCounts.untested})</option>
            <option value="validated">· Validated ({evalCounts.validated})</option>
            <option value="smoke_pass">· Smoke ({evalCounts.smoke_pass})</option>
            <option value="failed">· Failed ({evalCounts.failed})</option>
          </select>
        </div>

        <div className="policy-lab__library-count">
          {filteredPolicies.length} of {policies.length} policies
        </div>

        <div className="policy-lab__policy-list">
          {filteredPolicies.map((policy) => {
            const isSelected = policyKey(policy) === selectedKey;
            const isCompatible = policy.compatible !== false;
            return (
              <button
                key={policyKey(policy)}
                className={`policy-lab__policy ${isSelected ? 'policy-lab__policy--selected' : ''}`}
                onClick={() => setSelectedKey(policyKey(policy))}
              >
                <span className="policy-lab__policy-title">
                  <span>{policy.name}</span>
                  {policy.is_loaded && <span className="policy-lab__active-badge">Active</span>}
                </span>
                <span className="policy-lab__policy-meta">
                  <span>{policy.source}</span>
                  <span className={`policy-lab__eval policy-lab__eval--${evalStatusOf(policy)}`}>
                    {EVAL_LABEL[evalStatusOf(policy)]}
                    {policy.validation?.grid_pass ? ` ${policy.validation.grid_pass}` : ''}
                  </span>
                  <span className={isCompatible ? '' : 'policy-lab__danger'}>
                    {policy.obs_dim != null ? `${policy.obs_dim} obs` : 'obs unknown'}
                  </span>
                  <span>{policy.num_checkpoints || 0} ckpts</span>
                </span>
              </button>
            );
          })}
          {filteredPolicies.length === 0 && (
            <div className="policy-lab__empty">No policies match these filters.</div>
          )}
        </div>
      </aside>

      <main className="policy-lab__detail">
        {!selectedPolicy ? (
          <div className="policy-lab__empty">Select a policy to inspect its artifacts.</div>
        ) : (
          <>
            <header className="policy-lab__policy-header">
              <div>
                <div className="policy-lab__eyebrow">{selectedPolicy.source}</div>
                <h2>{selectedPolicy.name}</h2>
                <div className="policy-lab__summary">
                  <span>{selectedPolicy.algorithm}</span>
                  <span className={selectedPolicy.compatible === false ? 'policy-lab__danger' : ''}>
                    {selectedPolicy.obs_dim != null ? `${selectedPolicy.obs_dim} observations` : 'Observation shape unknown'}
                  </span>
                  <span>{selectedPolicy.requires_reset ? 'Sim resets on load' : 'Current sim shape'}</span>
                  <span>{selectedPolicy.size_mb} MB checkpoints</span>
                </div>
              </div>
              <button
                className="policy-lab__refresh"
                onClick={() => {
                  setSelectedKey(null);
                  listPolicies();
                }}
              >
                Refresh library
              </button>
            </header>

            <TagEditor policy={selectedPolicy} onSaved={listPolicies} />

            {selectedPolicy.compatible === false && (
              <div className="policy-lab__warning">
                {selectedPolicy.compatibility_error ||
                  `This ${selectedPolicy.obs_dim ?? 'unknown'}-observation policy cannot run in the current simulator.`}
              </div>
            )}
            {policyLoadError && <div className="policy-lab__warning">{policyLoadError}</div>}
            {detailStatus === 'error' && <div className="policy-lab__warning">{detailError}</div>}

            {detailStatus === 'loading' ? (
              <div className="policy-lab__empty">Reading checkpoints and videos...</div>
            ) : detail ? (
              <div className="policy-lab__artifact-grid">
                <section className="policy-lab__section policy-lab__section--checkpoints">
                  <div className="policy-lab__section-header">
                    <div>
                      <h3>Model Checkpoints</h3>
                      <span>{detail.checkpoint_count} files / {humanSize(detail.checkpoint_bytes)}</span>
                    </div>
                    <input
                      className="policy-lab__input policy-lab__input--compact"
                      type="search"
                      placeholder="Find checkpoint"
                      value={checkpointQuery}
                      onChange={(event) => {
                        setCheckpointQuery(event.target.value);
                        setCheckpointLimit(INITIAL_CHECKPOINT_LIMIT);
                      }}
                    />
                  </div>
                  <div className="policy-lab__artifact-list">
                    {checkpoints.slice(0, checkpointLimit).map((checkpoint) => {
                      const isLoaded = selectedPolicy.is_loaded &&
                        (selectedPolicy.loaded_checkpoint === checkpoint.name ||
                          policyCheckpoint === checkpoint.name);
                      const isLoading = loadingCheckpoint === checkpoint.name &&
                        policyLoadStatus === 'loading';
                      return (
                        <div
                          key={checkpoint.name}
                          className={`policy-lab__artifact ${isLoaded ? 'policy-lab__artifact--active' : ''}`}
                        >
                          <div className="policy-lab__artifact-copy">
                            <strong>{checkpoint.name}</strong>
                            <span>
                              {checkpoint.step != null ? `Step ${checkpoint.step.toLocaleString()} / ` : ''}
                              {humanSize(checkpoint.size_bytes)} / {formatDate(checkpoint.modified_iso)}
                            </span>
                          </div>
                          <button
                            className="policy-lab__test"
                            onClick={() => handleLoad(checkpoint.name)}
                            disabled={selectedPolicy.compatible === false || isLoaded || isLoading}
                            title={selectedPolicy.compatible === false
                              ? 'This checkpoint observation shape is unsupported'
                              : selectedPolicy.requires_reset
                                ? 'Load this checkpoint and reset the simulator automatically'
                                : 'Load this checkpoint in the simulator'}
                          >
                            {isLoading ? 'Loading' : isLoaded ? 'Loaded' : 'Test'}
                          </button>
                        </div>
                      );
                    })}
                    {checkpoints.length === 0 && (
                      <div className="policy-lab__empty">No matching checkpoints.</div>
                    )}
                  </div>
                  {checkpoints.length > checkpointLimit && (
                    <button
                      className="policy-lab__show-more"
                      onClick={() => setCheckpointLimit((limit) => limit + INITIAL_CHECKPOINT_LIMIT)}
                    >
                      Show {Math.min(INITIAL_CHECKPOINT_LIMIT, checkpoints.length - checkpointLimit)} more
                    </button>
                  )}
                </section>

                <section className="policy-lab__section policy-lab__section--videos">
                  <div className="policy-lab__section-header">
                    <div>
                      <h3>Episode Videos</h3>
                      <span>{detail.video_count} files / {humanSize(detail.video_bytes)}</span>
                    </div>
                    <input
                      className="policy-lab__input policy-lab__input--compact"
                      type="search"
                      placeholder="Find video"
                      value={videoQuery}
                      onChange={(event) => {
                        setVideoQuery(event.target.value);
                        setVideoLimit(INITIAL_VIDEO_LIMIT);
                      }}
                    />
                  </div>

                  <div className="policy-lab__video-preview">
                    {selectedVideo ? (
                      <>
                        <video
                          key={selectedVideo.url}
                          src={selectedVideo.url}
                          controls
                          preload="metadata"
                        />
                        <div className="policy-lab__video-caption">
                          <strong>{selectedVideo.name}</strong>
                          <span>{humanSize(selectedVideo.size_bytes)} / {formatDate(selectedVideo.modified_iso)}</span>
                        </div>
                      </>
                    ) : (
                      <div className="policy-lab__empty">No video selected.</div>
                    )}
                  </div>

                  <div className="policy-lab__video-list">
                    {videos.slice(0, videoLimit).map((video) => (
                      <button
                        key={video.name}
                        className={`policy-lab__video-item ${video.name === selectedVideoName ? 'policy-lab__video-item--selected' : ''}`}
                        onClick={() => setSelectedVideoName(video.name)}
                      >
                        <strong>{video.name}</strong>
                        <span>{humanSize(video.size_bytes)} / {formatDate(video.modified_iso)}</span>
                      </button>
                    ))}
                    {videos.length === 0 && (
                      <div className="policy-lab__empty">No matching episode videos.</div>
                    )}
                  </div>
                  {videos.length > videoLimit && (
                    <button
                      className="policy-lab__show-more"
                      onClick={() => setVideoLimit((limit) => limit + INITIAL_VIDEO_LIMIT)}
                    >
                      Show {Math.min(INITIAL_VIDEO_LIMIT, videos.length - videoLimit)} more
                    </button>
                  )}
                </section>
              </div>
            ) : null}
          </>
        )}
      </main>
    </div>
  );
}
