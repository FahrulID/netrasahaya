import mongoose, { Schema, Document, Model } from "mongoose";

// Define the interface for the Location document
interface ILocation extends Document {
  name: string;
  x: number;
  y: number;
}

// Define the schema for the Location model
const LocationSchema: Schema = new Schema({
  name: { type: String, required: true },
  x: { type: Number, required: true },
  y: { type: Number, required: true },
});

// Create the Location model
const Location: Model<ILocation> =
  mongoose.models.Location ||
  mongoose.model<ILocation>("Location", LocationSchema);

export default Location;
