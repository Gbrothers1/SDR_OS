import React, { useEffect, useState, useMemo, useRef, useCallback } from 'react';
import { createPortal } from 'react-dom';
import { useGenesis } from '../contexts/GenesisContext';
import '../styles/PolicyBrowserPanel.css';

const FILTER_TABS = [
  { key: 'all', label: 'All' },
  { key: 'PPO', label: 'PPO' },
  { key: 'BC', label: 'BC' },
];

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
          return (
            <button
              key={name}
              className={`ckpt-overflow__item ${isActive ? 'ckpt-overflow__item--active' : ''}`}
              onClick={() => onSelect(name)}
              disabled={isActive}
            >
              <span className="ckpt-overflow__item-name">{name}</span>
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
  const canLoad = isDir && !isActive && !isLoading;

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
          title={!isDir ? 'Standalone .pt files cannot be loaded directly (no cfgs.pkl)' : isActive ? 'Already loaded' : 'Load latest checkpoint'}
        >
          {isActive ? 'Loaded' : 'Load'}
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
    if (!query.trim()) return items;
    const q = query.trim().toLowerCase();
    return items.filter((p) =>
      (p.name || '').toLowerCase().includes(q) ||
      (p.path || '').toLowerCase().includes(q)
    );
  }, [policyList, filter, query]);

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
                ? 'No policies found in rl/checkpoints/'
                : 'No matching policies'}
            </div>
          ) : (
            filtered.map((policy) => (
              <PolicyCard
                key={policy.path}
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
