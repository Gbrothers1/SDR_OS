import React, { useEffect, useState, useMemo, useRef, useCallback } from 'react';
import { createPortal } from 'react-dom';
import { useGenesis } from '../contexts/GenesisContext';
import '../styles/PolicyBrowserPanel.css';

const FILTER_TABS = [
  { key: 'all', label: 'All' },
  { key: 'PPO', label: 'PPO' },
  { key: 'BC', label: 'BC' },
];

// Eval status (from run_registry.jsonl, attached by the backend as
// policy.validation). The "evaluated vs not-evaluated" axis plus the per-verdict
// breakdown. 'evaluated' is the union of the three verdict statuses.
const EVALUATED_STATUSES = ['validated', 'smoke_pass', 'failed'];
const EVAL_PILLS = {
  validated: { label: 'Validated', cls: 'policy-card__pill--good' },
  smoke_pass: { label: 'Smoke', cls: 'policy-card__pill--ok' },
  failed: { label: 'Failed', cls: 'policy-card__pill--bad' },
  untested: { label: 'Untested', cls: 'policy-card__pill--muted' },
};

const evalStatusOf = (policy) => policy?.validation?.status || 'untested';
const isCheckpointEvaluated = (policy, name) => {
  const ev = policy?.validation?.checkpoint_evals?.[name];
  return ev && (ev.status === 'eval_passed' || ev.visual === 'passed' || ev.grid_pass);
};

const timeAgo = (isoStr) => {
  const diff = Date.now() - new Date(isoStr).getTime();
  const mins = Math.floor(diff / 60000);
  if (mins < 60) return `${mins}m ago`;
  const hours = Math.floor(mins / 60);
  if (hours < 24) return `${hours}h ago`;
  const days = Math.floor(hours / 24);
  return `${days}d ago`;
};

const CheckpointList = ({ policy, loadedCheckpoint, onSelect }) => {
  return (
    <div className="ckpt-overflow">
      <div className="ckpt-overflow__header">
        <span className="ckpt-overflow__algo">{policy.algorithm}</span>
        <span className="ckpt-overflow__name">{policy.name}</span>
      </div>
      <div className="ckpt-overflow__list">
        {policy.checkpoints.map((name) => {
          const isActive = name === loadedCheckpoint;
          const ev = policy.validation?.checkpoint_evals?.[name];
          const evalled = isCheckpointEvaluated(policy, name);
          return (
            <button
              key={name}
              className={`ckpt-overflow__item ${isActive ? 'ckpt-overflow__item--active' : ''}`}
              onClick={() => onSelect(name)}
              disabled={isActive}
            >
              <span className="ckpt-overflow__item-name">{name}</span>
              {evalled && (
                <span className="ckpt-overflow__eval" title={`Evaluated${ev?.grid_pass ? ` — grid ${ev.grid_pass}` : ''}`}>
                  ✓ eval{ev?.grid_pass ? ` ${ev.grid_pass}` : ''}
                </span>
              )}
              {isActive && <span className="ckpt-overflow__badge">loaded</span>}
            </button>
          );
        })}
      </div>
    </div>
  );
};

const PolicyCard = ({ policy, isLoading, onLoad, onToggleCheckpoints, checkpointsOpen }) => {
  const isActive = policy.is_loaded;
  const isDir = policy.type === 'directory';
  const canBrowse = isDir && policy.num_checkpoints > 1;
  const isCompatible = policy.compatible !== false;
  const canLoad = isDir && isCompatible && !isActive && !isLoading;

  return (
    <div className={`policy-card ${isActive ? 'policy-card--active' : ''} ${isLoading ? 'policy-card--loading' : ''} ${checkpointsOpen ? 'policy-card--ckpt-open' : ''}`}>
      <div className="policy-card__main">
        <div className="policy-card__header">
          <span className="policy-card__algo">{policy.algorithm}</span>
          <span className="policy-card__name">{policy.name}</span>
          {isActive && <span className="policy-card__badge">Active</span>}
          {isLoading && <span className="policy-card__spinner" />}
        </div>
        <div className="policy-card__meta">
          {(() => {
            const pill = EVAL_PILLS[evalStatusOf(policy)] || EVAL_PILLS.untested;
            const grid = policy.validation?.grid_pass;
            return (
              <span className={`policy-card__pill ${pill.cls}`} title="Evaluation status (run registry)">
                {pill.label}{grid ? ` ${grid}` : ''}
              </span>
            );
          })()}
          {policy.validation?.eval_checkpoint && (
            <span className="policy-card__stat policy-card__stat--highlight" title="Evaluated checkpoint">
              eval&nbsp;{policy.validation.eval_checkpoint.replace('.pt', '')}
            </span>
          )}
          {(policy.tags || []).map((tag) => (
            <span key={tag} className="policy-card__tag" title="Tag (edit in LAB)">{tag}</span>
          ))}
          {policy.source && (
            <span className="policy-card__stat policy-card__stat--muted">{policy.source}</span>
          )}
          {policy.obs_dim != null && (
            <span className={`policy-card__stat ${isCompatible ? '' : 'policy-card__stat--error'}`}>
              {policy.obs_dim} obs{policy.requires_reset ? ' / reset' : ''}
            </span>
          )}
          {policy.loaded_checkpoint ? (
            <span className="policy-card__stat policy-card__stat--highlight">
              {policy.loaded_checkpoint}
            </span>
          ) : policy.latest_step != null ? (
            <span className="policy-card__stat">Step {policy.latest_step.toLocaleString()}</span>
          ) : null}
          {isDir && policy.num_checkpoints > 1 && (
            <span className="policy-card__stat">{policy.num_checkpoints} ckpts</span>
          )}
          <span className="policy-card__stat">{policy.size_mb} MB</span>
          <span className="policy-card__stat">{timeAgo(policy.modified_iso)}</span>
          {!isDir && <span className="policy-card__stat policy-card__stat--muted">file (no cfgs.pkl)</span>}
        </div>
      </div>
      <div className="policy-card__actions">
        {canBrowse && (
          <button
            className={`policy-card__action ${checkpointsOpen ? 'policy-card__action--active' : ''}`}
            onClick={() => onToggleCheckpoints(policy)}
            disabled={isLoading}
            title="Browse checkpoints"
          >
            Checkpoints
          </button>
        )}
        <button
          className="policy-card__action policy-card__action--primary"
          onClick={() => onLoad(policy)}
          disabled={!canLoad}
          title={!isDir
            ? 'Standalone .pt files cannot be loaded directly (no cfgs.pkl)'
            : !isCompatible
              ? `Unsupported observation layout (${policy.obs_dim ?? 'unknown'}); viewer supports crawl 225, jump 240, hurdle 250, walk 310/315`
              : isActive
                ? 'Already loaded'
                : 'Load latest checkpoint'}
        >
          {isActive ? 'Loaded' : isCompatible ? 'Load' : 'Unsupported'}
        </button>
      </div>
    </div>
  );
};

const PolicyBrowserPanel = ({ onExpandChange }) => {
  const {
    policyList,
    policyLoadStatus,
    policyLoadError,
    listPolicies,
    loadPolicy,
    genesisConnected,
    policyCheckpoint,
    envInfo,
    setDt,
  } = useGenesis();

  const [filter, setFilter] = useState('all');
  const [obsFilter, setObsFilter] = useState('all');
  const [evalFilter, setEvalFilter] = useState('all');
  const [query, setQuery] = useState('');
  const [loadingPath, setLoadingPath] = useState(null);
  const [expandedPolicy, setExpandedPolicy] = useState(null);
  const [dtInput, setDtInput] = useState(() => (envInfo?.dt ?? 0.02).toFixed(3));

  // Keep dt input in sync with envInfo updates
  useEffect(() => {
    if (envInfo?.dt != null) {
      setDtInput(Number(envInfo.dt).toFixed(3));
    }
  }, [envInfo?.dt]);

  // Fetch policies on mount and when genesis connects
  useEffect(() => {
    if (genesisConnected) {
      listPolicies();
    }
  }, [genesisConnected, listPolicies]);

  // Clear loading state when load completes
  useEffect(() => {
    if (policyLoadStatus === 'loaded' || policyLoadStatus === 'error') {
      setLoadingPath(null);
      // Refresh list so is_loaded / loaded_checkpoint update
      if (policyLoadStatus === 'loaded') listPolicies();
    }
  }, [policyLoadStatus, listPolicies]);

  // Notify parent of expand state
  useEffect(() => {
    if (onExpandChange) onExpandChange(!!expandedPolicy);
  }, [expandedPolicy, onExpandChange]);

  const filtered = useMemo(() => {
    let items = policyList;
    if (filter !== 'all') items = items.filter((p) => p.algorithm === filter);
    if (obsFilter === 'unknown') {
      items = items.filter((p) => p.obs_dim == null);
    } else if (obsFilter !== 'all') {
      items = items.filter((p) => Number(p.obs_dim) === Number(obsFilter));
    }
    if (evalFilter === 'evaluated') {
      items = items.filter((p) => EVALUATED_STATUSES.includes(evalStatusOf(p)));
    } else if (evalFilter !== 'all') {
      items = items.filter((p) => evalStatusOf(p) === evalFilter);
    }
    if (!query.trim()) return items;
    const q = query.trim().toLowerCase();
    return items.filter((p) =>
      (p.name || '').toLowerCase().includes(q) ||
      (p.source || '').toLowerCase().includes(q) ||
      (p.path || '').toLowerCase().includes(q) ||
      (p.tags || []).some((tag) => tag.toLowerCase().includes(q))
    );
  }, [policyList, filter, obsFilter, evalFilter, query]);

  const evalCounts = useMemo(() => {
    const c = { all: policyList.length, evaluated: 0, validated: 0, smoke_pass: 0, failed: 0, untested: 0 };
    policyList.forEach((p) => {
      const s = evalStatusOf(p);
      c[s] = (c[s] || 0) + 1;
      if (EVALUATED_STATUSES.includes(s)) c.evaluated += 1;
    });
    return c;
  }, [policyList]);

  const obsDimensions = useMemo(() => (
    [...new Set(
      policyList
        .map((policy) => policy.obs_dim)
        .filter((dimension) => dimension != null)
        .map(Number)
    )].sort((a, b) => a - b)
  ), [policyList]);

  const counts = useMemo(() => {
    const c = { all: policyList.length, PPO: 0, BC: 0 };
    policyList.forEach((p) => {
      if (p.algorithm === 'PPO') c.PPO++;
      else if (p.algorithm === 'BC') c.BC++;
    });
    return c;
  }, [policyList]);

  const handleLoad = (policy) => {
    setLoadingPath(policy.path);
    loadPolicy(policy.path);
  };

  const handleToggleCheckpoints = useCallback((policy) => {
    setExpandedPolicy((prev) =>
      prev && prev.path === policy.path ? null : policy
    );
  }, []);

  const handleSelectCheckpoint = useCallback((name) => {
    if (!expandedPolicy) return;
    setLoadingPath(expandedPolicy.path);
    loadPolicy(expandedPolicy.path, name);
    setExpandedPolicy(null);
  }, [expandedPolicy, loadPolicy]);

  const handleRefresh = () => {
    listPolicies();
  };

  const applyDt = useCallback(() => {
    const parsed = parseFloat(dtInput);
    if (!Number.isFinite(parsed)) return;
    setDt(parsed);
  }, [dtInput, setDt]);

  const handleDtKey = useCallback((e) => {
    if (e.key === 'Enter') {
      e.preventDefault();
      applyDt();
    }
  }, [applyDt]);

  const loadedName = policyCheckpoint || null;
  const portalTarget = document.getElementById('overflow-panel-right');

  return (
    <>
      <div className="policy-browser">
        <div className="policy-browser__header">
          <div>
            <div className="policy-browser__title">Policy Library</div>
            <div className="policy-browser__subtitle">Select a checkpoint directory to load</div>
          </div>
          <button
            className="policy-browser__refresh"
            onClick={handleRefresh}
            title="Rescan checkpoints"
          >
            Refresh
          </button>
        </div>

        {/* Filter tabs */}
        <div className="policy-browser__filters">
          <div className="policy-browser__tabs">
            {FILTER_TABS.map((tab) => (
              <button
                key={tab.key}
                className={`policy-browser__tab ${filter === tab.key ? 'policy-browser__tab--active' : ''}`}
                onClick={() => setFilter(tab.key)}
              >
                {tab.label} ({counts[tab.key] || 0})
              </button>
            ))}
          </div>
          <select
            className="policy-browser__obs-filter"
            value={obsFilter}
            onChange={(event) => setObsFilter(event.target.value)}
            aria-label="Filter policies by observation dimension"
          >
            <option value="all">All obs dims</option>
            {obsDimensions.map((dimension) => (
              <option key={dimension} value={dimension}>{dimension} obs</option>
            ))}
            <option value="unknown">Unknown obs</option>
          </select>
          <select
            className="policy-browser__obs-filter"
            value={evalFilter}
            onChange={(event) => setEvalFilter(event.target.value)}
            aria-label="Filter policies by evaluation status"
          >
            <option value="all">All eval ({evalCounts.all})</option>
            <option value="evaluated">Evaluated ({evalCounts.evaluated})</option>
            <option value="untested">Not evaluated ({evalCounts.untested})</option>
            <option value="validated">· Validated ({evalCounts.validated})</option>
            <option value="smoke_pass">· Smoke ({evalCounts.smoke_pass})</option>
            <option value="failed">· Failed ({evalCounts.failed})</option>
          </select>
        </div>

        <div className="policy-browser__search">
          <input
            className="policy-browser__input policy-browser__input--search"
            type="text"
            placeholder="Search name or path"
            value={query}
            onChange={(e) => setQuery(e.target.value)}
          />
          <div className="policy-browser__search-count">
            {filtered.length} / {policyList.length}
          </div>
        </div>

        {/* Status bar */}
        <div className="policy-browser__status">
          <div className="policy-browser__status-left">
            {loadedName && (
              <span className="policy-browser__loaded">
                <span className="policy-browser__loaded-dot" />
                {loadedName}
              </span>
            )}
          </div>
          <div className="policy-browser__status-right">
            <label className="policy-browser__label" htmlFor="policy-dt">dt</label>
            <input
              id="policy-dt"
              className="policy-browser__input"
              type="number"
              step="0.001"
              min="0.001"
              max="0.1"
              value={dtInput}
              onChange={(e) => setDtInput(e.target.value)}
              onKeyDown={handleDtKey}
              disabled={!genesisConnected}
              title="Simulation timestep (seconds)"
            />
            <button
              className="policy-browser__apply"
              onClick={applyDt}
              disabled={!genesisConnected}
              title="Apply timestep"
            >
              Set
            </button>
          </div>
        </div>

        {/* Error banner */}
        {policyLoadError && (
          <div className="policy-browser__error">
            {policyLoadError}
          </div>
        )}

        {/* Policy list */}
        <div className="policy-browser__list">
          {filtered.length === 0 ? (
            <div className="policy-browser__empty">
              {policyList.length === 0
                ? 'No policies found in configured checkpoint libraries'
                : 'No matching policies'}
            </div>
          ) : (
            filtered.map((policy) => (
              <PolicyCard
                key={policy.id || policy.path}
                policy={policy}
                isLoading={loadingPath === policy.path}
                onLoad={handleLoad}
                onToggleCheckpoints={handleToggleCheckpoints}
                checkpointsOpen={expandedPolicy && expandedPolicy.path === policy.path}
              />
            ))
          )}
        </div>
      </div>

      {expandedPolicy && portalTarget &&
        createPortal(
          <CheckpointList
            policy={expandedPolicy}
            loadedCheckpoint={expandedPolicy.loaded_checkpoint}
            onSelect={handleSelectCheckpoint}
          />,
          portalTarget
        )
      }
    </>
  );
};

export default PolicyBrowserPanel;
