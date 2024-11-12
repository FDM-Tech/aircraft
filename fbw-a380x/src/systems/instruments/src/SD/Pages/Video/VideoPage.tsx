import React from 'react';
import { PageTitle } from '../Generic/PageTitle';
import MapViewAircraft from './elements/MapViewWasmModule.wasm';
import '../../../index.scss';

export const VideoPage = () => (
  <PageTitle x={6} y={29}>
    VIDEO
  </PageTitle>
  <MapViewAircraft> </MapViewAircraft>
);
